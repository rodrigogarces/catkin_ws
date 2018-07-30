if (sim_call_type==sim_childscriptcall_initialization) then 
    leftJointHandle=simGetObjectHandle("dr12_leftJoint_")
    rightJointHandle=simGetObjectHandle("dr12_rightJoint_")
    bumperSensorHandle=simGetObjectHandle("dr12_bumperForceSensor_")
    pathHandle=simGetObjectHandle('dr12_pathPlanningPath')
    robotHandle=simGetObjectHandle('dr12_robot_')
    targetHandle=simGetObjectHandle('dr12_goalPos')
    pathPlanningHandle=simGetPathPlanningHandle('dr12_task')
    collidableForPathPlanning=simGetObjectHandle('dr12_collidableForPathPlanning')
    obstacles=simGetCollectionHandle('dr12_obstacles')
    desiredTargetPos={-99,-99}
    
    backwardModeUntilTime=0
    randomModeUntilTime=0
    pathCalculated=0 -- 0=not calculated, 1=beeing calculated, 2=calculated
    tempPathSearchObject=-1
    currentPosOnPath=0
    nominalVelocity=300*math.pi/180
    leftV=0
    rightV=0
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_actuation) then 
    targetP=simGetObjectPosition(targetHandle,-1)
    vv={targetP[1]-desiredTargetPos[1],targetP[2]-desiredTargetPos[2]}
    if (math.sqrt(vv[1]*vv[1]+vv[2]*vv[2])>0.01) then
        pathCalculated=0 -- We have to recompute the path since the target position has moved
        desiredTargetPos[1]=targetP[1]
        desiredTargetPos[2]=targetP[2]
    end
    
    
    currentTime=simGetSimulationTime()
    result,f,t=simReadForceSensor(bumperSensorHandle)
    if (result>0) then
        if (math.abs(f[2])>1) or (math.abs(f[3])>1) then
            backwardModeUntilTime=currentTime+3 -- 3 seconds backwards
        end
    end
    
    if (currentTime<backwardModeUntilTime) then
        -- Sensors detected something. We are too close to continue using a precalculated path, we navigate blindly a little bit back
        pathCalculated=0
    else
        rightV=0
        leftV=0
        if (pathCalculated==0) then
            if (simCheckCollision(obstacles,collidableForPathPlanning)~=1) then -- Make sure we are not colliding when starting to compute the path!
                if (tempPathSearchObject~=-1) then     
                    simPerformPathSearchStep(tempPathSearchObject,true) -- delete any previous temporary path search object
                end
                orientation=simGetObjectOrientation(robotHandle,-1)
                simSetObjectOrientation(robotHandle,-1,{0,0,orientation[3]}) -- Temporarily set the robot's orientation to be in the plane (the robot can slightly tilt back and forth)
                tempPathSearchObject=simInitializePathSearch(pathPlanningHandle,10,0.03) -- search for a maximum of 10 seconds
                simSetObjectOrientation(robotHandle,-1,orientation) -- Set the previous robot's orientation
                if (tempPathSearchObject~=-1) then
                    pathCalculated=1
                end
            else
                if (currentTime>randomModeUntilTime) then
                    randomModeUntilTime=currentTime+2 -- 2 seconds in random direction
                    randomVLeft=(-1+math.random()*2)*nominalVelocity
                    randomVRight=(-1+math.random()*2)*nominalVelocity
                end
            end
        else
            if (pathCalculated==1) then
                r=simPerformPathSearchStep(tempPathSearchObject,false)
                if (r<1) then
                    if (r~=-2) then
                        pathCalculated=0 -- path search failed, try again from the beginning
                        tempPathSearchObject=-1
                    end
                else
                    pathCalculated=2 -- we found a path
                    currentPosOnPath=0
                    tempPathSearchObject=-1
                end
            else
                l=simGetPathLength(pathHandle)
                r=simGetObjectPosition(robotHandle,-1)
                while true do
                    p=simGetPositionOnPath(pathHandle,currentPosOnPath/l)
                    d=math.sqrt((p[1]-r[1])*(p[1]-r[1])+(p[2]-r[2])*(p[2]-r[2]))
                    if (d>0.10)or(currentPosOnPath>=l) then
                        break
                    end
                    currentPosOnPath=currentPosOnPath+0.01
                end
                m=simGetObjectMatrix(robotHandle,-1)
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
            end
        end
    
    end
    
    if (currentTime<backwardModeUntilTime) then
        simSetJointTargetVelocity(leftJointHandle,-100*math.pi/180)
        simSetJointTargetVelocity(rightJointHandle,-50*math.pi/180)
    else
        if (currentTime<randomModeUntilTime) then
            simSetJointTargetVelocity(leftJointHandle,randomVLeft)
            simSetJointTargetVelocity(rightJointHandle,randomVRight)
        else
            simSetJointTargetVelocity(leftJointHandle,leftV)
            simSetJointTargetVelocity(rightJointHandle,rightV)
        end
    end
end 
    