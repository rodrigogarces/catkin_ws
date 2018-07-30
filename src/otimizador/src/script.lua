-- This example script is non-threaded (executed at each simulation pass)
-- The functionality of this script (or parts of it) could be implemented
-- in an extension module (plugin) and be hidden. The extension module could
-- also allow connecting to and controlling the real robot.

if (sim_call_type==sim_childscriptcall_initialization) then 
    -- First time we execute this script. 


    path_handle=simGetObjectHandle('Path')

    currentPosOnPath=0
    nominalVelocity=600*math.pi/180
    leftV=0
    rightV=0


    -- Make sure we have version 2.4.12 or above (the omni-wheels are not supported otherwise)
    v=simGetInt32Parameter(sim_intparam_program_version)
    if (v<20412) then
        simDisplayDialog('Warning','The YouBot model is only fully supported from V-REP version 2.4.12 and above.&&nThis simulation will not run as expected!',sim_dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=simGetObjectHandle('rollingJoint_fl')
    wheelJoints[2]=simGetObjectHandle('rollingJoint_rl')
    wheelJoints[3]=simGetObjectHandle('rollingJoint_rr')
    wheelJoints[4]=simGetObjectHandle('rollingJoint_fr')
    
    target=simGetObjectHandle('dummy_goal') -- alvo
    youBot=simGetObjectHandle('youBot')     -- robo    

    youBotRef=simGetObjectHandle('youBot_ref')
    tip=simGetObjectHandle('youBot_positionTip')
    target=simGetObjectHandle('youBot_positionTarget')
    armJoints={-1,-1,-1,-1,-1}
    for i=0,4,1 do
        armJoints[i+1]=simGetObjectHandle('youBotArmJoint'..i)
    end
    ui=simGetUIHandle('youBot_UI')
    simSetUIButtonLabel(ui,0,simGetObjectName(youBot)..' user interface') -- Set the UI title (with the name of the current robot)
    ik1=simGetIkGroupHandle('youBotUndamped_group')
    ik2=simGetIkGroupHandle('youBotDamped_group')
    ikFailedReportHandle=-1
    forwBackVelRange={-240*math.pi/180,240*math.pi/180}  -- min and max wheel rotation vel. for backward/forward movement
    leftRightVelRange={-240*math.pi/180,240*math.pi/180} -- min and max wheel rotation vel. for left/right movement
    rotVelRange={-240*math.pi/180,240*math.pi/180}       -- min and max wheel rotation vel. for left/right rotation movement

    forwBackVel=0
    leftRightVel=0
    rotVel=0
    initSizeFactor=simGetObjectSizeFactor(youBot) -- only needed if we scale the robot up/down
    
    -- desired joint positions, and desired cartesian positions:
    desiredJ={0,30.91*math.pi/180,52.42*math.pi/180,72.68*math.pi/180,0} -- when in FK mode
    for i=1,5,1 do
        simSetJointPosition(armJoints[i],desiredJ[i])
    end
    desiredPos={0,0,0} -- when in IK mode
    currentPos={0,0,0} -- when in IK mode
    ikMinPos={-0.5*initSizeFactor,-0.2*initSizeFactor,-0.3*initSizeFactor}
    ikRange={1*initSizeFactor,1*initSizeFactor,0.9*initSizeFactor}

    -- We compute the initial position and orientation of the tip RELATIVE to the robot base (because the base is moving)
    initialTipPosRelative=simGetObjectPosition(tip,youBotRef)--youBot)
    ikMode=false -- We start in FK mode
    maxJointVelocity=40*math.pi/180 
    maxPosVelocity=0.1*initSizeFactor
    previousS=initSizeFactor

    gripperCommunicationTube=simTubeOpen(0,'youBotGripperState'..simGetNameSuffix(nil),1)

    simExtROS_enableSubscriber('cmd_vel',1,simros_strmcmd_set_twist_command,-1,-1,'signal_cmd_vel')
    simExtROS_enableSubscriber('path_plan',1,simros_strmcmd_set_string_signal,-1,-1,'signal_path_plan')

end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_actuation) then 

print("signal_path_plan")
datap=simGetStringSignal('signal_path_plan')
if (datap) then
    floatTablep=simUnpackFloats(datap)
    simClearStringSignal('signal_path_plan')

    tl = table.getn(floatTablep) / 3

    ctrlPointsBufferTable = {}
    ptData = {}
    
    simCutPathCtrlPoints(path_handle,-1,simGetPathLength(path_handle))
    for i=1,tl do
        ptData[1] = floatTablep[3*(i-1)+1]
        ptData[2] = floatTablep[3*(i-1)+2]
        ptData[3] = floatTablep[3*(i-1)+3]
        ptData[4] = 0.0
        ptData[5] = 0.0
        ptData[6] = 0.0
        ptData[7] = 1.0
        ptData[8] = 0.0
        ptData[9] = 0
        ptData[10] = 1.0
        ptData[11] = 1.0
        
        --ctrlPointsBufferTable[i] = ptData[i]
        simInsertPathCtrlPoints(path_handle,0,i,1,ptData);

        print(floatTablep[i])  --linear x 
        print(floatTablep[i+1])  --linear y
        print(floatTablep[i+2])  --linear z
        print("-------------")
    end
end

    --TFs
    simExtROS_enablePublisher("/tf",1,simros_strmcmd_get_transform, simGetObjectHandle("base_link"),simGetObjectHandle("world_frame"), '/base_link%/odom')

    simExtROS_enablePublisher("/tf",1,simros_strmcmd_get_transform, simGetObjectHandle("base_front_link"),simGetObjectHandle("base_link"), '/base_front_link%/base_link')
    --simExtROS_enablePublisher("true_position",1,simros_strmcmd_get_object_pose ,simGetObjectHandle("youBot_frame"),simGetObjectHandle("World_ref"), '')
    simExtROS_enablePublisher("/tf",1,simros_strmcmd_get_transform, simGetObjectHandle("Hokuyo_URG_04LX_UG01_ROS_ref"), simGetObjectHandle("base_front_link"), '/Hokuyo_URG_04LX_UG01_ROS%/base_front_link') --hier eine exakte transformation aber 90 grad verdreht

    simExtROS_enablePublisher("pose",1,simros_strmcmd_get_object_pose,simGetObjectHandle("youBot_ref"),simGetObjectHandle("world_frame"), '')

    l=simGetPathLength(path_handle)
    r=simGetObjectPosition(youBot,-1)
    t=simGetObjectPosition(target,-1)

    result,distanceData=simCheckDistance(youBot,target,0)

    --drt=math.sqrt((t[1]-r[1])*(t[1]-r[1])+(t[2]-r[2])*(t[2]-r[2]))
    print("distancia: " .. distanceData[7]);
    if (result == 2) and (distanceData[7] < 0.5) then
        simSetJointTargetVelocity(wheelJoints[1],0.0)
        simSetJointTargetVelocity(wheelJoints[2],0.0)
        simSetJointTargetVelocity(wheelJoints[3],0.0)
        simSetJointTargetVelocity(wheelJoints[4],0.0)
    else
        while true do
            p=simGetPositionOnPath(path_handle,currentPosOnPath/l)
            d=math.sqrt((p[1]-r[1])*(p[1]-r[1])+(p[2]-r[2])*(p[2]-r[2]))
            if (d>0.10)or(currentPosOnPath>=l) then
                break
            end
            currentPosOnPath=currentPosOnPath+0.01
        end

        m=simGetObjectMatrix(youBot,-1)
        m=simGetInvertedMatrix(m)
        p=simMultiplyVector(m,p)
        -- Now p is relative to the robot
        a=math.atan2(p[2],p[1])
        if (a>=0)and(a<math.pi*0.5) then
            rightV=nominalVelocity
            leftV=nominalVelocity*(1-2*a/(math.pi*0.5))
        end
        if (a>=math.pi*0.5) then
            leftV=-nominalVelocity
            rightV=nominalVelocity*(1-2*(a-math.pi*0.5)/(math.pi*0.5))
        end
        if (a<0)and(a>-math.pi*0.5) then
            leftV=nominalVelocity
            rightV=nominalVelocity*(1+2*a/(math.pi*0.5))
        end
        if (a<=-math.pi*0.5) then
            rightV=-nominalVelocity
            leftV=nominalVelocity*(1+2*(a+math.pi*0.5)/(math.pi*0.5))
        end

        -- Now apply the desired wheel velocities:
        --simSetJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
        --simSetJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
        --simSetJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
        --simSetJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)

        simSetJointTargetVelocity(wheelJoints[1],leftV)
        simSetJointTargetVelocity(wheelJoints[2],leftV)
        simSetJointTargetVelocity(wheelJoints[3],rightV)
        simSetJointTargetVelocity(wheelJoints[4],rightV)
    end

    
end

