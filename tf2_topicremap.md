# TF2

tf2 Transform Library is used to keep track of coordinate frames over time. It allows robot components to understand their position and orientation relative to each other, enabling proper spatial awareness and movement.

TF2 tools help visualize, analyze, and debug coordinate frames and transforms within the TF2 system. 
<br><br>

### Main tf2 commands :  

create a pdf file in the pwd showing the TF tree at that moment :  
`ros2 run tf2_tools view_frames` 
<br><br>


To display transforms between 2 frames :  
`ros2 run tf2_ros tf2_echo [source_frame] [target_frame]`
<br><br>


Monitor transform update delays (useful for debugging delays in TF updates) :  
`ros2 run tf2_ros tf2_monitor`
<br><br>

### Theory :  
*Add notes picture here*  
<br><br>

Broadcasting- publishing to /tf & /tf_static
Listening- subscribing to /tf & tf_static
<br>

Transforms - 2 types:  -Static: no relative change  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;-Dynamic: move w.r.t time
<br>

Transforms go from child to parent link.   
<br>
___
<br>
<br>

# Topic Remapping

~/topic : private topic  
&nbsp;&nbsp;&nbsp;topic : relative topic  
&nbsp;&nbsp;/topic : absolute topic  

<br>

### Syntax to remap topics :  
*(--ros-args syntax will works only for newer ros2 versions and not humble)*
<br><br>

**CLI**  
`--ros-args --remap <from_topic_name>:=<to_topic_name>`  

<br>

**Gazebo Plugin**  
```
<ros>  
    <arguments> --ros-args --remap <from_topic_name>:=<to_topic_name>  
</ros>
```  

<br>

**Launch File**  
- Python based   
    ```
    Node(  
        remappings = [ ('from_topic' , 'to_topic') ]  
    )
    ```

- XML based
    ```
    <node>
        <remap from="orignal_topic" to="new_topic" />
    </node>
    ```










                










