if (sim_call_type==sim_childscriptcall_initialization) then 
    bubbleRobBase=simGetObjectAssociatedWithScript(sim_handle_self)
    ctrl=simGetUIHandle("bubbleCtrl")
    simSetUIButtonLabel(ctrl,0,simGetObjectName(bubbleRobBase).." speed")
    leftMotor=simGetObjectHandle("leftMotor")
    rightMotor=simGetObjectHandle("rightMotor")
    noseSensor=simGetObjectHandle("sensingNose")
    minMaxSpeed={50*math.pi/180,300*math.pi/180}
    backUntilTime=-1 -- Tells whether bubbleRob is in forward or backward mode
    floorSensorHandles={-1,-1,-1}
    floorSensorHandles[1]=simGetObjectHandle("leftSensor")
    floorSensorHandles[2]=simGetObjectHandle("middleSensor")
    floorSensorHandles[3]=simGetObjectHandle("rightSensor")
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_sensing) then 

end 

if (sim_call_type==sim_childscriptcall_actuation) then 
    speed=minMaxSpeed[1]+(minMaxSpeed[2]-minMaxSpeed[1])*simGetUISlider(ctrl,3)/1000
    result=simReadProximitySensor(noseSensor)
    if (result>0) then backUntilTime=simGetSimulationTime()+4 end
    
    -- read the line detection sensors:
    sensorReading={false,false,false}
    for i=1,3,1 do
        result,data=simReadVisionSensor(floorSensorHandles[i])
        if (result>=0) then
            sensorReading[i]=(data[11]<0.3) -- data[11] is the average of intensity of the image
        end
    end
    
    -- compute left and right velocities to follow the detected line:
    rightV=speed
    leftV=speed
    if sensorReading[1] then
        leftV=0.03*speed
    end
    if sensorReading[3] then
        rightV=0.03*speed
    end
    if sensorReading[1] and sensorReading[3] then
        backUntilTime=simGetSimulationTime()+2
    end
    
    if (backUntilTime<simGetSimulationTime()) then
        -- When in forward mode, we simply move forward at the desired speed
        simSetJointTargetVelocity(leftMotor,leftV)
        simSetJointTargetVelocity(rightMotor,rightV)
    else
        -- When in backward mode, we simply backup in a curve at reduced speed
        simSetJointTargetVelocity(leftMotor,-speed/2)
        simSetJointTargetVelocity(rightMotor,-speed/8)
    end
end 
