# Softstarter-using-ARM-STM32F411

This is a project intended for the matter of power eletronics and microprocessed systems II. The goal is to develop a softstarter with the following requirements:
- Acceleration and deceleration ramps should rang from 5 to 50 seconds;
- If there is a lack of energy, at its return the system must remain off until it is activated again;
- The circuit must have over current protection at 200% of the rated current, immediately inhibiting the power supply;
- During the acceleration ramp the current can't exceed 150% of the rated one;
- Be capable of measuring the voltage and current rms;
- Contains an interface to allow the user change the settings;
- Use RTOS on the ARM, with the least of 3 tasks.

**To Do:**
- [x] Build the zero crossing detector
- [X] Implement the triggering to triac circuit
- [X] Be able to vary the ramps time
- [ ] Bypass relay
- [ ] Eletrical break
- [ ] Schematic
- [ ] measure current throught a current transformer
