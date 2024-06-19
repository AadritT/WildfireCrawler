This is the Repository for the Arduino code for the Wildfire Crawler Project

1. Motor Control - This is the Arduino Uno code that controls the motors to both move the arms and the wheels. The Uno is also used to trigger the other systems: The camera classification as well as the LoRa communication. 

2. Camera Classification - This is the ESP32-CAM code that controls the camera functionality. The code also includes the machine learning library (from Edge Impulse) to classify the image taken. The code returns the classification result to the Motor Control System.

3. LoRa Communication - This is the MKR1010 code that controls the LoRa radio to send the classification result along with crawler metadata (ID and GNSS location) to The Things Network.

System Execution Flow: 
1. When the Crawler is turned on, the Motor Control will raise the arms to secure the wheels in place.
2. The Motor Control then rotates the wheels to move to a new location.
3. Once done moving, the Motor Control sends a signal to the Camera Classification.
4. The Camera Classification then takes a picture of the below landscape and runs the classification model on the captured image.
5. Once finished, the Camera Classification sends the result to the Motor Control, which is a probability value represented as an Integer from 0 - 100.
6. When received, the Motor Control sends this value to the LoRa Communication.
7. The LoRa Communication establishes a connection between the radio and The Things Network.
8. When the connection is confirmed, the LoRa Communication sends the probability, a Crawler ID already present in the code, and the GNSS location to The Things Network as a CBOR packet.
9. The LoRa Communication then sends a signal to the Motor Control to indicate the packet transmission completion.
10. When received, the Motor Control repeats the cycle from Step 2.
