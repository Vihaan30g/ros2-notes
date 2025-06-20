

## What is DDS and how does ros 2 uses it

DDS (Data Distribution Service) is a middleware protocol designed for real-time, scalable, and reliable data exchange in distributed systems. It operates using a publisher-subscriber model, where data producers and consumers communicate through topics. Participants in the system are discovered through a process called discovery, which has two main types: simple discovery and discovery server. After discovery, data is transferred using a chosen transport mechanism, such as UDP, TCP, or others.

ROS 2 uses DDS as its underlying layer to implement its various communication types — such as topics, services, and actions. All these are built on top of the same DDS publisher-subscriber mechanism, but ROS 2 configures them differently to achieve the desired communication patterns. Additionally, ROS 2 provides QoS (Quality of Service) settings that allow developers to customize communication behavior to suit specific application needs.
<br>

## Peer to peer communication

Peer-to-peer (P2P) communication is a model in which two or more devices, known as peers, communicate directly with each other without the need for a central server or intermediary. In this model, all peers have equal roles and capabilities, allowing data to be exchanged directly between them. This results in faster and more efficient data transfer. Since there is no central point of failure, the system remains functional even if some peers go offline, as the remaining peers can work independently.
<br>


## Reasons for Dropping ROS Master in ROS 2:  

Decentralization:  
If the master failed, the whole system would stop working. ROS 2 removes this central part, making the system more reliable and fault-tolerant.  

Better Scalability and Speed:  
Without a central master, Nodes talk directly to each other, which makes communication faster and reduces delays.

DDS better supports real-time processes, it also simplifies overall structure.    
<br>



## **How Services Are Implemented in ROS 2**

###  How It Works:  
* ROS 2 defines **two topics** behind the scenes:  
  1. **Request topic**: Client publishes requests here.
  2. **Response topic**: Server sends replies here.
<br>


## **How Actions Are Implemented in ROS 2**

### How It Works:  
ROS 2 actions use **five DDS topics** internally:  
1. **Goal** – Client sends a goal.  
2. **Result** – Server sends back the final result.  
3. **Feedback** – Server provides progress updates.  
4. **Cancel** – Client can cancel a goal.  
5. **Status** – Keeps track of goal states.  
<br>
<br>

---


ROS 1 used TCP even on same computer, but it didn't matter because the speed difference was negligible b/w shared memory transport and TCP on same system. For furthur faster communications, ROS1 used shared mem transport.

ROS 2 uses shared memory transport for communication within the sytem.

For communication b/w 2 different systems, ROS 2 uses the RTPS (Real-Time Publish-Subscribe) protocol from DDS. Data is serialized first then transmitted. QoS is used to configure the communication. This can then be sent over several systems like - WiFI, ethernet, etc.  

#### Shared memory system ? :   
Instead of this:  
Node A → Copy data → Send over network → Node B receives → Copy again  

You do this(shared mem transport):  
Node A & Node B → Access the same memory space → Just pass a pointer  
<br>
<br>


## Daemon
A daemon is needed because querying the DDS graph can be slow.  It starts a background node, subscribes to DDS discovery data, caches the information, and speeds up command-line queries.  
The daemon only adopts the communication settings from the terminal where it was started and these do not update. To use it with any updated settings, it must be stopped and then restarted with the new settings.
<br>



In ROS 2, the IP address of nodes is set at startup. If the computer's IP changes (e.g., connecting to a new network), nodes won't automatically detect the new IP. To address this:
- Restart nodes to update their networking settings with the new IP.
- Use static IP addresses and reserve them on your network to prevent changes.
- Ensure all networks are active and connected before launching ROS 2 nodes.
<br>
<br>



## DISCOVERY

#### Simple Discovery
Participants (nodes) send messages over the network and then all participants share discovery information (such as topics and services) with all other participants. This means that all participants get information about all other participants irrelevant of whether or not they need that information.


#### Discovery Server
When a participant (node) starts, it checks in with the servers to share its discovery information and request what information it needs. In this way, participants only establish communication as necessary.
<br>
<br>



## QoS Compatibility
Subscriptions request a QoS profile that is the “minimum quality” that it is willing to accept, and publishers offer a QoS profile that is the “maximum quality” that it is able to provide. Connections are only made if every policy of the requested QoS profile is not more stringent than that of the offered QoS profile. All the policies must be compatible, else no data would be communicated.
<br>

## QoS main policies


### 1. Reliability  
Determines whether the system ensures message delivery.  
**Options:**  
RELIABLE: Guarantees delivery. Retries if needed.  
BEST_EFFORT: No guarantees — messages may be lost.  


### 2. Durability  
Decides whether new subscribers can receive messages that were sent before they joined.  
**Options:**  
VOLATILE: Only receive new messages after subscribing.  
TRANSIENT_LOCAL: Also receive the last sent message, even if you join late.  

  
### 3. History  
Controls how many past messages are stored in the queue.  
**Options:**  
KEEP_LAST: Store only the latest N messages (controlled by depth).  
KEEP_ALL: Try to store all messages (until memory runs out).  

  

### 4. Depth  
Defines how many messages to keep when using KEEP_LAST history.  
**Options:**  
Any positive integer (e.g., 10)  



### 5. Deadline  
Sets a time limit for how often a message should arrive. If not received within this limit, the sys will notify us.  
**Options:**  
Duration (e.g., 500ms)  


### 6. Lifespan  
Specifies how long a message is valid. After that time, the message is dropped and never delivered.  
**Options:**  
Duration (e.g., 2s)  
 

### 7. Liveliness
Defines how a node proves it's still alive and publishing.  
**Options:**  
AUTOMATIC: System sends heartbeats.  
MANUAL_BY_TOPIC: Your node must manually “ping” to show it's alive.


### 8. Liveliness Lease Duration
How long to wait before assuming a publisher is dead (if no heartbeat received).  
**Options:**  
Duration (e.g., 5s)
<br>



## QoS Compatibility
Subscriptions request a QoS profile that is the “minimum quality” that it is willing to accept, and publishers offer a QoS profile that is the “maximum quality” that it is able to provide. Connections are only made if every policy of the requested QoS profile is not more stringent than that of the offered QoS profile. All the policies must be compatible, else no data would be communicated.
<br>
<br>

**Compatibility of Reliability QoS Policies**

| Publisher | Subscription | Compatible |
|---|---|---|
| Best effort | Best effort | Yes |
| Best effort | Reliable | No |
| Reliable | Best effort | Yes |
| Reliable | Reliable | Yes |


**Compatibility of Durability QoS Policies**

| Publisher | Subscription | Compatible | Result |
|---|---|---|---|
| Volatile | Volatile | Yes | New messages only |
| Volatile | Transient local | No | No communication |
| Transient local | Volatile | Yes | New messages only |
| Transient local | Transient local | Yes | New and old messages |


**Compatibility of Deadline QoS Policies**

| Publisher | Subscription | Compatible |
|---|---|---|
| Default | Default | Yes |
| Default | x | No |
| x | Default | Yes |
| x | x | Yes |
| x | y (where y > x) | Yes |
| x | y (where y < x) | No |


**Compatibility of Liveliness QoS Policies**

| Publisher | Subscription | Compatible |
|---|---|---|
| Automatic | Automatic | Yes |
| Automatic | Manual by topic | No |
| Manual by topic | Automatic | Yes |
| Manual by topic | Manual by topic | Yes |


**Compatibility of Lease Duration QoS Policies**

| Publisher | Subscription | Compatible |
|---|---|---|
| Default | Default | Yes |
| Default | x | No |
| x | Default | Yes |
| x | x | Yes |
| x | y (where y > x) | Yes |
| x | y (where y < x) | No |

<br>
<br>


## QoS Events
QoS Events in ROS 2 are notifications or callbacks that trigger when something related to (QoS) happens. 
For example: A deadline was missed, A publisher or subscriber became inactive, A liveliness guarantee was violated, Messages were lost or dropped.



















