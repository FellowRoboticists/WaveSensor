# -*- ruby -*-
# WaveSensor Arduino sketch
#
# Copyright (c) 2013,2014 Dave Sieh
#
# See LICENSE.txt for details.

# Put the parent directory on the Ruby Load path
$: << File.dirname(File.dirname(__FILE__))

# Bring in the task support
require 'arduino-tasks/tasks'
include ArduinoTasks

BASE_DIR = '..'

env = ArduinoEnvironment.new BASE_DIR

LIBS = [
  library('VCNL4000'),
  library('pspc_support'),
  library('WaveHC', 'wavehc')
]

create_all_library_tasks env, LIBS, :default


#LIB_DIR = 'lib'
#BASE_DIR = '..'

#WAVEHC_LIB = File.join(LIB_DIR, 'WaveHC')
#WAVEHC_SRC = 'wavehc/WaveHC'

#directory WAVEHC_LIB do
  #cp_r File.join(BASE_DIR, WAVEHC_SRC), LIB_DIR
#end

#LIBS = %w{ VCNL4000 }

#LIBS.each do | lib |
  #directory File.join(LIB_DIR, lib) do
    #cp_r File.join(BASE_DIR, lib, LIB_DIR, lib), LIB_DIR
  #end

  #task :default => File.join(LIB_DIR, lib)
#end

#task :default => WAVEHC_LIB


