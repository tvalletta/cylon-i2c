/*
 * cylon-gpio
 * http://cylonjs.com
 *
 * Copyright (c) 2013 The Hybrid Group
 * Licensed under the Apache 2.0 license.
*/


'use strict';

require('cylon');
require('./blinkm');
require('./hmc6352');
require('./mpl115a2');
require('./bmp180');
require('./lcd');
require('./l3g4200d')

module.exports = {
  driver: function(opts) {
    switch (opts.name) {
      case 'blinkm':
        return new Cylon.Drivers.I2C.BlinkM(opts);
      case 'hmc6352':
        return new Cylon.Drivers.I2C.Hmc6352(opts);
      case 'mpl115a2':
        return new Cylon.Drivers.I2C.Mpl115A2(opts);
      case 'bmp180':
        return new Cylon.Drivers.I2C.Bmp180(opts);
      case 'lcd':
        return new Cylon.Drivers.I2C.LCD(opts);
      case 'l3g4200d':
        return new Cylon.Drivers.I2C.L3G4200D(opts);
      default:
        return null;
    }
  },

  register: function(robot) {
    Logger.debug("Registering i2c BlinkM driver for " + robot.name);
    robot.registerDriver('cylon-i2c', 'blinkm');

    Logger.debug("Registering i2c HMC6352 driver for " + robot.name);
    robot.registerDriver('cylon-i2c', 'hmc6352');

    Logger.debug("Registering i2c MPL115A2 driver for " + robot.name);
    robot.registerDriver('cylon-i2c', 'mpl115a2');

    Logger.debug("Registering i2c BMP180 driver for " + robot.name);
    robot.registerDriver('cylon-i2c', 'bmp180');

    Logger.debug("Registering i2c LCD driver for " + robot.name);
    robot.registerDriver('cylon-i2c', 'lcd');

    Logger.debug("Registering i2c L3G4200D driver for " + robot.name);
    robot.registerDriver('cylon-i2c', 'l3g4200d');
  }
};
