# AS_Challange

 #### `Working Students of Team 11`
1.  `Siyan Li`
2.  `Bingkun Huang`
3.  `Zhenjiang Li`
4.  `Haowen Shi`
5.  `Weili Tang`

## Controller 
To run controller:

`roslaunch controller_pkg controller.launch`

In this part,  

the `controller` subscribes two topics: `current_state` and `desired state` with "pos" & "twists".

the `controller` calculates the `error` so that we can use the `mav_comm` package to calculate `rotor_speed`

the `controller` publisches one topic: `Rotor speed` 
