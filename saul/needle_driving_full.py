import os
import time

import prepare_needle_insertion

import PedriatricSimulator

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
time.sleep(1)
project_path = os.getcwd()
sim.save_simulation_state(project_path + "/before_needle_insertion_1.simstate")
sim.disconnect()

import needle_insertion_1

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
time.sleep(1)
project_path = os.getcwd()
sim.save_simulation_state(project_path + "/before_needle_driving_1.simstate")
sim.disconnect()

import needle_driving_1

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
time.sleep(1)
project_path = os.getcwd()
sim.save_simulation_state(project_path + "/before_needle_insertion_2.simstate")
sim.disconnect()

import needle_insertion_2

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
time.sleep(1)
project_path = os.getcwd()
sim.save_simulation_state(project_path + "/before_needle_driving_2.simstate")
sim.disconnect()

import needle_driving_2

sim = PedriatricSimulator.PediatricSimulator()
sim.connect("127.0.0.1")
time.sleep(1)
project_path = os.getcwd()
sim.save_simulation_state(project_path + "/before_needle_extraction.simstate")
sim.disconnect()

import prepare_needle_extraction
