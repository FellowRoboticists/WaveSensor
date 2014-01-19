# -*- ruby -*-
# WaveSensor Arduino sketch
#
# Copyright (c) 2013 Dave Sieh
#
# See LICENSE.txt for details.

LIB_DIR = 'lib'
BASE_DIR = '..'
BLE_BASE_DIR = '../BLEShield/Arduino/libraries'

WAVEHC_LIB = File.join(LIB_DIR, 'WaveHC')
WAVEHC_SRC = 'wavehc/WaveHC'

directory WAVEHC_LIB do
  cp_r File.join(BASE_DIR, WAVEHC_SRC), LIB_DIR
end

LIBS = %w{ VCNL4000 }

LIBS.each do | lib |
  directory File.join(LIB_DIR, lib) do
    cp_r File.join(BASE_DIR, lib, LIB_DIR, lib), LIB_DIR
  end

  task :default => File.join(LIB_DIR, lib)
end

BLE_LIBS = %w{ Nordic_BLE RBL_BLEShield }

BLE_LIBS.each do | lib |
  directory File.join(LIB_DIR, lib) do
    cp_r File.join(BLE_BASE_DIR, lib), LIB_DIR
  end

  task :default => File.join(LIB_DIR, lib)
end


task :default => WAVEHC_LIB


