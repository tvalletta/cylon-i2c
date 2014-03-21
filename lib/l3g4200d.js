/**
 * Created by validity on 3/19/14.
 */
/*
 * L3G4200D I2C Three Axis Gyroscope from ST
 * http://cylonjs.com
 *
 * Copyright (c) 2013-14 The Hybrid Group
 * Licensed under the Apache 2.0 license.
 */

'use strict';

require('./cylon-i2c');

var namespace = require('node-namespace');

namespace("Cylon.Drivers.I2C", function() {

  return this.L3G4200D = (function(klass) {

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

    subclass(L3G4200D, klass);

    function L3G4200D() {
      L3G4200D.__super__.constructor.apply(this, arguments);
      this.address = 0x69;

      initRegisters(function() {
        Logger.debug('init complete');
      });
    }

    function initRegisters(callback) {
      this.connection.i2cWrite(this.address, L3G4200D_REGISTER_CTRL_REG1, parseInt('00001111', 2), function(err) {
        if (err) console.log('ERROR: ' + err);
        else {
          this.connection.i2cWrite(this.address, L3G4200D_REGISTER_CTRL_REG2, parseInt('00000000', 2), function(err) {
            if (err) console.log('ERROR: ' + err);
            else {
              this.connection.i2cWrite(this.address, L3G4200D_REGISTER_CTRL_REG3, parseInt('00001000', 2), function(err) {
                if (err) console.log('ERROR: ' + err);
                else {
                  this.connection.i2cWrite(this.address, L3G4200D_REGISTER_CTRL_REG4, parseInt('00110000', 2), function(err) {
                    if (err) console.log('ERROR: ' + err);
                    else {
                      this.connection.i2cWrite(this.address, L3G4200D_REGISTER_CTRL_REG5, parseInt('00000000', 2), function(err) {
                        if (err) console.log('ERROR: ' + err);
                        else {
                          callback();
                        }
                      });
                    }
                  });
                }
              });
            }
          });
        }
      });
    }

    L3G4200D.prototype.commands = function() {
      return ['getGyroValues'];
    };

    L3G4200D.prototype.start = function(callback) {
      this.readCoefficients(callback);
    };

    L3G4200D.prototype.getGyroValues = function(callback) {
      if (callback == null) {
        callback = null;
      }
      this.getPT(callback);
    };

    L3G4200D.prototype.readCoefficients = function(callback) {
      var thiz = this;
      return this.connection.i2cRead(this.address, L3G4200D_REGISTER_OUT_X_H, 15, function(err, data) {
        if(err) callback(err);
        else {
          thiz.xMSB = toSShort(data[0],  data[1]);
          thiz.xLSB = toSShort(data[2],  data[3]);
          thiz.ac3 = toSShort(data[4],  data[5]);
          thiz.ac4 = toUShort(data[6],  data[7]);
          thiz.ac5 = toUShort(data[8],  data[9]);
          thiz.ac6 = toUShort(data[10], data[11]);

          _this.b1 = toSShort(data[12], data[13]);
          _this.b2 = toSShort(data[14], data[15]);

          _this.mb = toSShort(data[16], data[17]);
          _this.mc = toSShort(data[18], data[19]);
          _this.md = toSShort(data[20], data[21]);

          //Logger.debug("AC1: " + _this.ac1);
          //Logger.debug("AC2: " + _this.ac2);
          //Logger.debug("AC3: " + _this.ac3);
          //Logger.debug("AC4: " + _this.ac4);
          //Logger.debug("AC5: " + _this.ac5);
          //Logger.debug("AC6: " + _this.ac6);
          //Logger.debug("B1: " + _this.b1);
          //Logger.debug("B2: " + _this.b2);
          //Logger.debug("MB: " + _this.mb);
          //Logger.debug("MC: " + _this.mc);
          //Logger.debug("MD: " + _this.md);

          callback();
          _this.device.emit('start');
        }
      });
    }

    return L3G4200D;

  })(Cylon.Driver);
});
