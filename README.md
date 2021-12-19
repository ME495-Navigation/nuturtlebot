Here are the ideas:

1. Integrate this into the nuturtle repository


catkin_make as a normal ros package

then

rosrun nuturtle create_image to make the raspberry pi image.
      
rosrun nuturtle write_image
    Options:
      username: username for the user (defaults to msr)
      password: password for the user (if blank will be prompted)
      
      - This defaults to an open ad-hoc network
      SSID: ssid of the network
      password: password of the network
      
      OR:
      path to network manager profile (will be scrubbed of mac address)
      
      hostname: the hostname for the computer
      robotdns: set to the robotdns server, or leave blank to not use robotdns
