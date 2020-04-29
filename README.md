![build](https://github.com/mit-drl/e-vent/workflows/build/badge.svg)

# MIT E-Vent: A Low Cost Emergency Ventilator Controller

![MIT E-Vent Prototype](https://user-images.githubusercontent.com/17116105/80438039-39f32580-88d1-11ea-8a16-f9bce3a209bf.jpg)

**This code is provided for reference use only.** Our goal is to help others understand the function of our prototype system, with regards to: delivering tidal volumes, breaths per minute and I:E ratios; visible and audible alarm functions; and the user interface. Assist control, to detect patient breathing, is implemented, but is still a work in progress. (Synchronizing with a human is non-trivial.) These features are only a baseline and others will likely wish to add more safeties and features.
 
_Caution: Any group working to develop ventilation or other medical device must conduct significant hardware and software validation and testing to identify and mitigate fault conditions. This is essential to ensure patient safety._

Please see [https://e-vent.mit.edu/](https://e-vent.mit.edu/) for a high level description of the complete system.


### Volume Control Mode Definitions
The following waveform diagram is a visual explanation of the key cycle parameters used in [e-vent.ino](e-vent.ino).

      |                                                                    
      |     |<- Inspiration ->|<- Expiration ->|                           
    P |                                                                    
    r |     |                 |                |                           
    e |             PIP -> /                                  /            
    s |     |             /|                   |             /|            
    s |                  / |__  <- Plateau                  / |__          
    u |     |           /     \                |           /     \         
    r |                /       \                          /       \        
    e |     |         /         \              |         /         \       
      |              /           \                      /           \      
      |     |       /             \            |       /             \     
      |            /               \                  /               \    
      |     |     /                 \          |     /                     
      |          /                   \              /                      
      |     |   /                     \        |   /                       
      |        /                       \          /                        
      |       /                         \        /                         
      | _____/                  PEEP ->  \______/                          
      |___________________________________________________________________ 
             |             |  |          |      |                          
             |<- tIn ----->|  |          |      |                   Time   
             |<- tHoldIn ---->|          |      |                          
             |<- tEx ------------------->|      |                          
             |<- tPeriod ---------------------->|                          
             |                                                             
                                                                           
         tCycleTimer                                                       

