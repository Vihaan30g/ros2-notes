
##**Week 3**

**This is the modified version of week_1 package, the nodes of this pkg includes manually written QoS profiles.**<br>
There are 2 nodes(publisher and subscriber). Publisher sends integers(1,2,...10) on topic. 1st five numbers are sent with a 1 sec gap in between, other five are sent at a 2 sec delay.  
I did this because i wanted to see how deadline-QoS works, so i set deadline duration to 1.5 sec hoping that i would get warning after 5th integer. But there was no difference. Then i found that python doesn't have a built-in method to log the warnings sent by deadline_missed callbacks. So the best way i could find to gewt the warnings diplayed in python itself was to find out the delay b/w the last message and current message, and compare with the duration of deadline. If its greater than the duration, then log a warning message.  
<br>
Thank you
