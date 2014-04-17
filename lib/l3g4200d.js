/*
 * L3G4200D I2C Three Axis Gyroscope from ST
 * http://cylonjs.com
 *
 * Copyright (c) 2013-14 The Hybrid Group
 * Licensed under the Apache 2.0 license.
 */

'use strict';

require('./cylon-i2c');
var Promise = require('bluebird');
var namespace = require('node-namespace');

namespace("Cylon.Drivers.I2C", function() {

  var L3G4200D_REGISTER_WHO_AM_I      = 0x0F;   // R  11010011  DEVICE IDENTIFICATION
  var L3G4200D_REGISTER_CTRL_REG1     = 0x20;   // RW 00000111  DATA RATE, BANDWIDTH, POWER, AXIS ENABLE
  var L3G4200D_REGISTER_CTRL_REG2     = 0x21;   // RW 00000000  HIGH PASS FILTER
  var L3G4200D_REGISTER_CTRL_REG3     = 0x22;   // RW 00000000  INTERRUPT AND BOOT STATUS
  var L3G4200D_REGISTER_CTRL_REG4     = 0x23;   // RW 00000000  SCALE, TEST, AND SPI MODE
  var L3G4200D_REGISTER_CTRL_REG5     = 0x24;   // RW 00000000  HIGH PASS FILTER ENABLE
  var L3G4200D_REGISTER_REFERENCE     = 0x25;   // R  -output-  REFERENCE / DATA CAPTURE
  var L3G4200D_REGISTER_OUT_TEMP      = 0x26;   // R  -output-  OUT TEMPERATURE
  var L3G4200D_REGISTER_STATUS_REG    = 0x27;   // R  -output-
  var L3G4200D_REGISTER_OUT_X_L       = 0x28;   // R  -output-
  var L3G4200D_REGISTER_OUT_X_H       = 0x29;   // R  -output-
  var L3G4200D_REGISTER_OUT_Y_L       = 0x2A;   // R  -output-
  var L3G4200D_REGISTER_OUT_Y_H       = 0x2B;   // R  -output-
  var L3G4200D_REGISTER_OUT_Z_L       = 0x2C;   // R  -output-
  var L3G4200D_REGISTER_OUT_Z_H       = 0X2D;   // R  -output-
  var L3G4200D_REGISTER_FIFO_CTRL_REG = 0x2E;   // RW 00000000
  var L3G4200D_REGISTER_FIFO_SRC_REG  = 0x2F;   // R  -output-
  var L3G4200D_REGISTER_INT1_CFG      = 0x30;   // RW 00000000
  var L3G4200D_REGISTER_INT1_SRC      = 0x31;   // R  -output-
  var L3G4200D_REGISTER_INT1_TSH_XH   = 0x32;   // RW 00000000
  var L3G4200D_REGISTER_INT1_TSH_XL   = 0x33;   // RW 00000000
  var L3G4200D_REGISTER_INT1_TSH_YH   = 0x34;   // RW 00000000
  var L3G4200D_REGISTER_INT1_TSH_YL   = 0x35;   // RW 00000000
  var L3G4200D_REGISTER_INT1_TSH_ZH   = 0x36;   // RW 00000000
  var L3G4200D_REGISTER_INT1_TSH_ZL   = 0x37;   // RW 00000000
  var L3G4200D_REGISTER_INT1_DURATION = 0x38;   // RW 00000000

  /**
   * If the SDO is connected to the voltage use SDO_TO_VOLTAGE for the
   * address.  If it is connected to the ground use SDO_TO_GROUND.  This
   * allows for two addressable gyroscopes on a single device.
   * @type {Number}
   */
  var SDO_TO_VOLTAGE                  = 0x69;   // -- 1101001 | 105
  var SDO_TO_GROUND                   = 0x68;   // -- 1101000 | 104

  var CTRL_REG1_INIT_VALUE            = 0x1F;   // -- 00001111 | 15
  var CTRL_REG2_INIT_VALUE            = 0x00;   // -- 00000000 | 00
  var CTRL_REG3_INIT_VALUE            = 0x08;   // -- 00001000 | 08
  var CTRL_REG4_INIT_VALUE            = 0x30;   // -- 00110000 | 48
  var CTRL_REG5_INIT_VALUE            = 0x00;   // -- 00000000 | 00

  return this.L3G4200D = (function(klass) {
    subclass(L3G4200D, klass);

    function L3G4200D() {
      L3G4200D.__super__.constructor.apply(this, arguments);

      this.gyro = {
        address: SDO_TO_VOLTAGE,
        calibration: {
          samples: 500,          // STMicro doc recommends 50
          sigma_mult: 3          // As recommended in STMicro doc
        },
        zeroRate: [],
        threshold: [],
        raw: [],
        dps: []
//        heading: [0, 0, 0],
//        lastTime: process.hrtime()
      }
    }

    L3G4200D.prototype.commands = function() {
      return [
        'pollGyroValues',
        'stopGyroValues',
        'setGyroSensitivity250',
        'setGyroSensitivity500',
        'setGyroSensitivity2000'
      ];
    };

    L3G4200D.prototype.start = function(callback) {
      Logger.debug('Attempting to start L3G4200D...');
      var inst = this;

      whoAmI.call(inst)
        .then(function() { return initRegisters.call(inst); })
        .then(function() { return calibrateGyro.call(inst); })
//      .then(function() { return testCalibration.call(inst); })
        .then(function() {
          Logger.debug('L3G4200D started.');
          callback();
          inst.device.emit('start');
        })
        .catch(function(e) {
          Logger.error('Failed to start L3G4200D: ', e);
          Logger.error(e.stack);
        });
    };

    L3G4200D.prototype.pollGyroValues = function(callback) {
      var inst = this;
      var gyro = this.gyro;
      inst.polling = true;

      function recurse() {
        if (inst.polling) {
          updateGyroValues.call(inst)
            .then(function() {
              if (callback) {
                callback(gyro.dps);
              }
              recurse();
            });
        }
      }
      recurse();
    };

    L3G4200D.prototype.stopGyroValues = function() {
      this.polling = false;
    };

//    L3G4200D.prototype.updateHeadings = function() {
//      Logger.debug();
//      Logger.debug('Updating Headings...');
//      var deltaT = this.getDeltaT();
//
//      for (var i = 0; i < 3; ++i) {
//        this.heading[i] -= (this.gyroDPS[i] * deltaT) / 1000000;
//      }
//    };

//    L3G4200D.prototype.getDeltaT = function() {
//      Logger.debug();
//      Logger.debug('Get DeltaT...(nanoseconds)');
//      var diff = process.hrtime(this.lastTime);
//      var deltaT = diff[0] * 1e9 + diff[1];
//
//      this.lastTime = process.hrtime();
//      Logger.debug('Got DeltaT: ' + deltaT);
//      return deltaT;
//    };


//    L3G4200D.prototype.getGyroValues = function(callback) {
//      var out = {
//        x: this.gyroRaw[0],
//        y: this.gyroRaw[1],
//        z: this.gyroRaw[2]
//      };
//      if (callback) callback(out);
//      return out;
//    };

//    L3G4200D.prototype.getGyroHeading = function(callback) {
//      var out = {
//        x: this.heading[0],
//        y: this.heading[1],
//        z: this.heading[2]
//      };
//      if (callback) callback(out);
//      return out;
//    };

//    L3G4200D.prototype.onGyroValue = function(callback) {
//      if (!this.polling) this.pollGyroValues();
//      this.gyroValueListener = callback;
//    };

//    L3G4200D.prototype.onGyroHeading = function(callback) {
//      if (!this.polling) this.pollGyroValues();
//      this.gyroHeadingListener = callback;
//    };

//    L3G4200D.prototype.getGyroValues = function(callback) {
//      var out = {
//        x: this.gyroRaw[0],
//        y: this.gyroRaw[1],
//        z: this.gyroRaw[2]
//      };
//      callback(out);
//      return out;
//    };

    L3G4200D.prototype.setGyroSensitivity250 = function() {
      this.gyro.dpsPerDigit = 0.00875;
      return i2cPromiseWrite.call(this, L3G4200D_REGISTER_CTRL_REG4, [0x80]);
    };

    L3G4200D.prototype.setGyroSensitivity500 = function() {
      this.gyro.dpsPerDigit = 0.0175;
      return i2cPromiseWrite.call(this, L3G4200D_REGISTER_CTRL_REG4, [0x90]);
    };

    L3G4200D.prototype.setGyroSensitivity2000 = function() {
      this.gyro.dpsPerDigit = 0.07;
      return i2cPromiseWrite.call(this, L3G4200D_REGISTER_CTRL_REG4, [0xA0]);
    };

//    L3G4200D.prototype.printDPS = function() {
//      Logger.debug('DPS x: ' + this.gyroDPS[0] + ' y: ' + this.gyroDPS[1] + ' z: ' + this.gyroDPS[1]);
//    };

//    L3G4200D.prototype.printHeadings = function() {
//      Logger.debug('Heading x: ' + this.heading[0] + ' y: ' + this.heading[1] + ' z: ' + this.heading[1]);
//    };


    // --- Private Functions ---

    /**
     * Checks the WHO_AM_I register.  If the device does not identify itself
     * appropriately an error is returned.
     *
     * IMPORTANT: Requires context to be set to an instance of the driver
     *
     * @returns {Promise}
     */
    function whoAmI() {
      var inst = this;
      return new Promise(function(resolve, reject) {
        i2cPromiseRead.call(inst, L3G4200D_REGISTER_WHO_AM_I, 1)
          .then(function(data) {
            if (data[0] === 0xD3) resolve(data[0]);
            else reject('Device at address is not L3G4200D');
          })
          .catch(function(err) { reject(err) });
      });
    }

    /**
     * Initialize the L3G4200D registers.  This turns on the sensor and sets
     * initial configuration values.
     *
     * IMPORTANT: Requires context to be set to an instance of the driver
     *
     * @returns {Promise}
     */
    function initRegisters() {
      var inst = this;
      return new Promise(function(resolve, reject) {
        i2cPromiseWrite.call(inst, L3G4200D_REGISTER_CTRL_REG1, [CTRL_REG1_INIT_VALUE])
          .then(function() { return i2cPromiseWrite.call(inst, L3G4200D_REGISTER_CTRL_REG2, [CTRL_REG2_INIT_VALUE]) })
          .then(function() { return i2cPromiseWrite.call(inst, L3G4200D_REGISTER_CTRL_REG3, [CTRL_REG3_INIT_VALUE]) })
          .then(function() { return i2cPromiseWrite.call(inst, L3G4200D_REGISTER_CTRL_REG4, [CTRL_REG4_INIT_VALUE]) })
          .then(function() { return i2cPromiseWrite.call(inst, L3G4200D_REGISTER_CTRL_REG5, [CTRL_REG5_INIT_VALUE]) })
          .then(function() { return inst.setGyroSensitivity250() })
          .then(function() { resolve() })
          .catch(function(err) { reject(err) });
      });
    }

    /**
     * Calibrate the gyroscope.  This step is important for steady readings that
     * do not drift over time.
     *
     * IMPORTANT: Requires context to be set to an instance of the driver
     *
     * @returns {Promise}
     */
    function calibrateGyro() {
      var inst = this;
      var gyro = this.gyro;
      var calib = gyro.calibration;
      var cTimer = process.hrtime();
      var raws = [[], [], []];

      return new Promise(function(resolve, reject) {
        var promises = [];
        for (var i = 0; i < calib.samples; ++i) {
          promises.push(new Promise(function (resolve, reject) {
            updateRawValues.call(inst, 50)
              .then(function() {
                raws[0].push(gyro.raw[0]);
                raws[1].push(gyro.raw[1]);
                raws[2].push(gyro.raw[2]);
                resolve();
              })
              .catch(function(err) {
                Logger.error('Calibrate Gyro Error: ' + err);
                reject(err);
              });
            }
          ));
        }
        Promise.all(promises)
          .then(function() {
            for (var i = 0; i < 3; ++i) {
              gyro.zeroRate[i] = zeroRate(raws[i]);
              Logger.debug()
              gyro.threshold[i] = stDev(raws[i]) * calib.sigma_mult;
            }
            var diff = process.hrtime(cTimer);
            Logger.debug('Gyro calibrated using ' + calib.samples + ' samples in ' + ((diff[0] * 1e9) + diff[1]) + 'ns');
            resolve();
          })
          .catch(function(err) {
            Logger.error('L3G4200D calibrateGyro ERROR: ' + err);
            reject(err);
          });
      });
    }

    /**
     * Calculate useful values from the raw data.
     *
     * IMPORTANT: Requires context to be set to an instance of the driver
     *
     * @param delay
     * @returns {Promise}
     */
    function updateGyroValues() {
      var inst = this;
      var gyro = this.gyro;
      var deltaGyro = [0, 0, 0];

      return new Promise(function(resolve, reject) {
        updateRawValues.call(inst, 3)
          .then(function() {
            for (var i = 0; i < 3; ++i) {
              deltaGyro[i] = gyro.raw[i] - gyro.zeroRate[i];
              if (Math.abs(deltaGyro[i]) < gyro.threshold[i])
                deltaGyro[i] = 0;
              gyro.dps[i] = gyro.dpsPerDigit * deltaGyro[i];
            }
            resolve(deltaGyro);
          })
          .catch(function(err) {
            Logger.error('L3G4200D updateGyroValues ERROR: ' + err);
            reject(err);
          });
      });
    }

    /**
     * Gets the raw values from the gyro when they become available
     *
     * IMPORTANT: Requires context to be set to an instance of the driver
     *
     * @param delay
     * @returns {Promise}
     */
    function updateRawValues(delay) {
      var inst = this;
      return new Promise(function(resolve, reject) {
        function waitForStatus(ready) {
          i2cPromiseRead.call(inst, L3G4200D_REGISTER_STATUS_REG, 1)
            .then(function(data) {
//              if (data[0] & 0x40) { Logger.debug('missed'); }
              if (data[0] & 0x08) { ready(); }
              else setTimeout(waitForStatus(ready), delay);
            })
            .catch(function(err) { ready(err); });
        }
        waitForStatus(function(err) {
          if (err) reject(err);
          else resolve(readCoefficients.call(inst));
        });
      });
    }

    /**
     * Get the raw values from the gyro
     *
     * IMPORTANT: Requires context to be set to an instance of the driver
     *
     * @returns {Promise}
     */
    function readCoefficients() {
      var inst = this;
      var gyro = this.gyro;

      return new Promise(function(resolve, reject) {
        Promise.all([
          i2cPromiseRead.call(inst, L3G4200D_REGISTER_OUT_X_L, 1),
          i2cPromiseRead.call(inst, L3G4200D_REGISTER_OUT_X_H, 1),
          i2cPromiseRead.call(inst, L3G4200D_REGISTER_OUT_Y_L, 1),
          i2cPromiseRead.call(inst, L3G4200D_REGISTER_OUT_Y_H, 1),
          i2cPromiseRead.call(inst, L3G4200D_REGISTER_OUT_Z_L, 1),
          i2cPromiseRead.call(inst, L3G4200D_REGISTER_OUT_Z_H, 1)
        ]).then(function(val) {
            gyro.raw[0] = toSShort(val[0][0], val[1][0]);
            gyro.raw[1] = toSShort(val[2][0], val[3][0]);
            gyro.raw[2] = toSShort(val[4][0], val[5][0]);
            resolve();
            inst.device.emit('start');
          })
          .catch(function(err) {
            Logger.debug('L3G4200D readCoefficients ERROR: ' + err);
            reject();
          });
      });
    }

    /**
     * Runs the calibration and writes the zero and threshold rates to the
     * debug log
     *
     * IMPORTANT: Requires context to be set to an instance of the driver
     *
     * @returns {Promise}
     */
    function testCalibration() {
      var inst = this;
      return new Promise(function(resolve, reject) {
        Logger.debug('Test Calibration...');

        calibrateGyro.call(inst)
          .then(function() {
            for (var i = 0; i < 3; ++i) {
              Logger.debug('gyro zero rate: [' + i + ']; ' + thiz.gyroZeroRate[i]);
              Logger.debug('gyro threshold rate: [' + i + ']; ' + thiz.gyroThreshold[i]);
            }
            resolve();
          })
          .catch(function(err) {
            Logger.debug('L3G4200D testCalibration ERROR: ' + err);
            reject(err);
          });
      });
    }


    // --- Utility Functions ---

    function toSShort(b1, b2) {
      return ((((b1) | (b2 << 8)) << 16) >> 16);
    }

    function i2cPromiseWrite(reg, val) {
      var inst = this;
      return new Promise(function(resolve, reject) {
        inst.connection.i2cWrite(inst.gyro.address, reg, val,
          function(err) {
            if (err) reject(err);
            else resolve();
          }
        );
      });
    }

    function i2cPromiseRead(reg, len) {
      var inst = this;
      return new Promise(function(resolve, reject) {
        inst.connection.i2cRead(inst.gyro.address, reg, len,
          function(err, data) {
            if (err) reject(err);
            else resolve(data);
          }
        );
      });
    }

    function zeroRate(a) {
      var s = 0;
      for (var i = 0; i < a.length; ++i) {
        s += a[i];
      }
      return s / a.length;
    }

    function stDev(a) {
      var n = a.length;
      var i = n, m, v, x = 0, y = 0;
      while (i) {
        v = a[--i];
        x += v*v;  // sum squares
        y += v;    // sum terms
      }
      return Math.sqrt( (x - y*y/n) / n );
    }

    return L3G4200D;
  })(Cylon.Driver);
});