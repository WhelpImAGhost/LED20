# ECE 411 - LED20
### Team 1
- Brad Glaubitz
- Chris Kane-Pardy
- Caleb Monti
- Henry Sanders



## Project Overview
The purpose of the LED20 project is to add a fun and interesting spin to modern D20s (20 sided dice) for various tabletop games. Rather than a standard D20 which simply shows which face is rolled, the LED20 will illuminate the face that was rolled, and play special sound effects when a 1 or 20 is rolled. This added flare will spice up any tabletop game, and can be customized to your liking! [^1]



## Important Files and Folders Directory
This repository holds all design and development documents that were either assigned or created out of necessity throughout this project, including the Product Design Specification and Bill of Materials. However, for someone interested in creating their own copy of the LED20, some of these files and folders are not required. So, we have compiled a list of the files and folders that are necessary for understanding and replicating this project



### Links to files and folders
- [Full Project Description](./Project%20Info%20and%20Requirements/Project%20Information%20and%20Requirements.pdf): A more in-depth explanation of the project details and required features
- [Functional Decomposition](./Functional%20Decomposition/): Other high-detail files which explain the inner hardware and software functions and connections
- [Bill of Materials](./BOMs/Bill%20of%20Materials%20v4.xlsx): The list of required components to recreate the circuit
- [3D Model Files](./CAD/3D%20Model/): Iterations of development for the 3D printed D20 enclosure and charging base
- [PCB Development Files](./CAD/Schematics%20and%20PCB%20Layouts/): Requierd files for modifying or fabricating the carrier circuit board
- [Code](./LED20_Code/): Most recent iteration of the code, developed using ESP-IDF for the ESP32-C6



[^1]: Customization must take place before the die is sealed, as there is not a current way to interface with the processor after the enclosure is fully sealed. Please make any changes to LED Color, 1 & 20 Tones, and Sleep States before sealing your die.