# neo_sick_field

Convenient way to select a SICK scanner *field set* by toggling relayboard relays to a specific scanner input case

## Getting started

Within **SICK CDS** software
1. Define some *Field Sets*
1. Set up *Cases* with respective *Input conditions*

then
1. Adjust `config/mapping.yaml` to refect your SICK settings and Neobotix relayboard wiring
1. `rosrun` the node

Exemplary *mapping.yaml*
```yaml
sick_inputs2neo_relay: {A: 2, B: 3}
sick_field-cases:
  15: {A: 0, B: 0}
  10: {A: 1, B: 0}
  nil: {A: 0, B: 1}
  park: {A: 1, B: 1}
```
results in 
```sh
$ rosservice list | grep neo_sick_field
/neo_sick_field/10
/neo_sick_field/15
/neo_sick_field/nil
/neo_sick_field/park
```

You can call to activate those scanner field sets
```sh
$ rosservice call /neo_sick_field/park
success: True
message: ''
```
now check the param server
```sh
$ rosparam get /neo_sick_field/field 
park
```

## API

+ Parameter server
  - ~field  
  Name (key) of last successfully set field case
+ Services
  - ~\<x>  
  Provides a service **std_srvs.Trigger** for each entry in *mapping.sick_field-cases*.  
  Those will call respective `/relayboard_v2/set_relay` for the scanner field case.
+ Publishes
  - nothing
+ Subscribes
  - none
