# BuoyancyEngine


The buoyancy engine is built with two watertight enclosures and external mounting support. These two watertight enclosure kits are assembled by PMMA tubes and the aluminum alloy flanges. 

The upper part, the electrical enclosure, includes one 2.4GHz antenna, one customized PCB, one pressure sensor,one peristaltic pump, two 9V batteries and other electrical components. The enclosure on the bottom is mainly used for storing the water drived by the peristaltic pump.


<p align="center">
    Rendered Buoyancy Engine <br />
    <img height="200" src="https://github.com/winnieay/BuoyancyEngine/assets/88380759/7a8c1507-93f6-4c0b-8b05-36906b9a37ac" >
    <img height="200" src="https://github.com/winnieay/BuoyancyEngine/assets/88380759/687129bf-da87-4b1c-bbc2-6734dbadcb98" ><br />
</p>



<p align="center">
    Buoyancy Engine Working Example <br />
   <img width="778" alt="Screenshot 2024-02-25 at 13 37 47" src="https://github.com/winnieay/BuoyancyEngine/assets/88380759/fed39d2c-db8e-4868-8259-6bbebb4b7d76"><br />
</p>


Specification:

1. Watertight Enclosure

Thickness: 5mm

Outer Dimension: 90mm

Length: (Upper)250mm, (Lower)100mm

2. Bouyancy Engine

Height: 620mm

Dimension: 120mm

Mechanism

As peristaltic pumps have no check valves to clog, they excel at driving fluids into lower pressure systems. Therefore, a peristaltic pump would be used to manipulate the motion of the engine: Sinking and Floating. Water is pumped through a flexible silicone tube in a peristaltic motion. Rollers are attached to a rotor that is controlled by a motor. As the rotor turns, the rollers pinch the tubing to force the water through. When the tube is not compressed, the water would be driven into the enclosure or pumped out the enclosure.

To indicate the position of the engine, a pressure sensor is one of the critical components in the engine. The engine could self-identify its position by taking advantage of the relationship between the fluid pressure and depth.

Data Transmission

For the communication between the ground station and the engine, a short-range wireless technology standard, Bluetooth, would be used to transmit and receive data. To avoid  improper connection, master-slave mode would be set up in the bluetooth modules. They are only allowed to be paired with each other and keep requiring re-pairing if they disconnect. A 2.4GHz antenna would increase signal strength to guarantee the data transmission even with long distance.

Buoyancy Support and Mounting Design

As the engine is positively buoyant, certain weights would be added into the lower enclosure to achieve slight positive buoyancy. Extra weights could be added or mounted to the 3D printed support with zip ties. It provides a flexible approach for buoyancy tuning while the engine is deployed into distinct environments.

To prevent electrical shock, the battery mount is designed to fix the position of the batteries onto the customized PCB. And the customized PCB would be mounting onto the upper aluminum alloy flange. 
