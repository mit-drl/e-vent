![build](https://github.com/mit-drl/e-vent/workflows/build/badge.svg)

# MIT E-Vent: A Low Cost Emergency Ventilator Controller

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

