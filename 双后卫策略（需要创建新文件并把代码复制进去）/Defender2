//后卫2的任务安排
//负责 右 半场的防守 
option(Defender2)
{
	const Vector2f ball = theBallModel.estimate.position;   //定义球的相对位置
	const Vector2f globalBall = Transformation::robotToField(theRobotPose, ball);   //定义球的绝对位置
	const float a=globalBall.x();  
	const float b=globalBall.y();
	const float m=theFieldDimensions.xPosOwnGroundline;  //-4500
	const float p=theRobotPose.translation.x();
	const float q=theRobotPose.translation.y();
	const float theta=atan(b/(std::abs(a-m)));
	const float x0=(b*b*m-b*q*(a-m)+(a-m)*(a-m)*p)/(b*b+(a-m)*(a-m));
	const float y0=(b*b*q+b*(a-m)*p-m*b*(a-m))/(b*b+(a-m)*(a-m));
	const Vector2f weizhi = Transformation::fieldToRobot(theRobotPose, Vector2f(x0,y0));
    std::vector<Obstacle> theobstacles = theObstacleModel.obstacles;//当前障碍物的信息。
	
	int turnInfo=0;
	//700;   标准场最小值
	//2500;  标准场最大值
	float xiao=700;   
	float da=3500;  
	
	//出场的初始位置：学校球场是800    标准球场是1300
	float chuchang=800;
	
	std::vector<Teammate> teammates = theTeammateData.teammates;
	int count = teammates.size();
	float distance = 5000.f;
	for (int i = 0; i < count;i++)
	{
		if(teammates[i].number==3)
		{
			distance = teammates[i].ball.estimate.position.norm();
			break;  
		}
	}
	//踢球位置  c1为x c2为y
	float c1;
	float c2;
	
	//球场左边是Y大于0,右边是Y小于0
    //自己半场是X小于0,敌方半场是X大于0 
	//加30 减50  700～2500的范围内 
	//减是让球到45.f位置 45f对应的是左脚if(theobstacles[i].type == Obstacle::someRobot)
    
	initial_state(start)  //开始状态，包括了 [球解围后回到初始位置] [判断此刻的优先级 以选择defender1/2]
	{
		
		transition
		{
			if (libCodeRelease.timeSinceBallWasSeen() > 5000)
				goto searchForBall;
			
			int theNumberOfObstacle;	
			theNumberOfObstacle  = theObstacleModel.obstacles.size();	
			int j=0;
            for (int i = 0;i<theNumberOfObstacle;i++)
				{	
					if (theObstacleModel.obstacles[i].center.x()<300 
					&& std::abs(theObstacleModel.obstacles[i].center.y())<200)      //判断前方有障碍
					{
						j++;
						break;
					}							
				}
				
			if(globalBall.x()>-1500.f && globalBall.x()<0.f && j==0)
				goto dajiao;
				
            if (theRobotPose.translation.x() > (theFieldDimensions.xPosOwnPenaltyArea+xiao ) 
			&& theRobotPose.translation.x() < (theFieldDimensions.xPosOwnPenaltyArea+da )
			&& globalBall.x() <0 
			&& distance>ball.norm())
                goto defend1;
				
			if (theRobotPose.translation.x() > (theFieldDimensions.xPosOwnPenaltyArea+xiao ) 
			&& theRobotPose.translation.x() < (theFieldDimensions.xPosOwnPenaltyArea+da )
			&& globalBall.x() <0 )
				goto defend2;
				
            if ((globalBall.x() < (theFieldDimensions.xPosOwnPenaltyArea+xiao)
			&& globalBall.x() > (theFieldDimensions.xPosOwnPenaltyArea))
			||(globalBall.x() < (theFieldDimensions.xPosOwnPenaltyArea)
			&&std::abs(globalBall.y()) > theFieldDimensions.yPosLeftPenaltyArea))                       
                goto sideKick;  
		} 
		action
		{
			LookForward();
			WalkToTarget(Pose2f(0.f, 0.8f, 0.8f), Transformation::fieldToRobot(theRobotPose,Vector2f(theFieldDimensions.xPosOwnPenaltyMark-100, theFieldDimensions.yPosLeftPenaltyArea-400.f)));
		}
	}
	
	state(searchForBall)   //找球状态
	{
		transition
		{
			if(turnInfo==0)
			{
			if (libCodeRelease.timeSinceBallWasSeen() < 200)
				goto start;
			if(state_time > 12000)
				goto start;
			}
			
		}
		action
		{
			if(turnInfo==1)
				WalkAtSpeedPercentage(Pose2f(0.5f, 0.f, 0.f));   //逆时针转
			else if(turnInfo==2)	
				WalkAtSpeedPercentage(Pose2f(-0.5f, 0.f, 0.f));  //顺时针转
			else if(state_time<4000)
				LookRound();
			else			
            {
				LookForward();
				if(theRobotPose.translation.y()>0)    
				   WalkAtSpeedPercentage(Pose2f(-0.5f, 0.f, 0.f));
				else
				   WalkAtSpeedPercentage(Pose2f(0.5f, 0.f, 0.f));         
			}
			turnInfo=0;
		}
	}
	
	state(defend1)     //前往球的位置抢球
	{
		transition
		{
			if (libCodeRelease.timeSinceBallWasSeen() > 5000)
				goto searchForBall;
				
			if (globalBall.x() < (theFieldDimensions.xPosOwnPenaltyArea + da)
			&& globalBall.x() > theRobotPose.translation.x()
			&& globalBall.x() > theFieldDimensions.xPosOwnPenaltyArea)
				goto runToBallAndKick;
                
            if (globalBall.x() < theRobotPose.translation.x()
			&& globalBall.x()>theFieldDimensions.xPosOwnPenaltyArea
			&& globalBall.x() < (theFieldDimensions.xPosOwnPenaltyArea + da))
                goto sideKick;
			
			if (theRobotPose.translation.x()>theFieldDimensions.xPosOwnPenaltyArea+da ||globalBall.x() > 0)
				goto start;
		}
		action
		{
            LookForward();
            if (globalBall.x() > (theFieldDimensions.xPosOwnPenaltyArea + da ))
            {
			Vector2f tempPos(theFieldDimensions.xPosOwnPenaltyArea+800.f, globalBall.y()*((theFieldDimensions.xPosOwnGroundline-theFieldDimensions.xPosOwnPenaltyArea-800.f) / (theFieldDimensions.xPosOwnGroundline - globalBall.x())));
            WalkToTarget(Pose2f(0.1f, 0.8f, 0.8f), Transformation::fieldToRobot(theRobotPose,tempPos));
            }
           else
		   {
               WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), ball);
		   }
        }
    }
	
	state(defend2)     //前往球的射门线上防守
	{
		transition
		{
			if (libCodeRelease.timeSinceBallWasSeen() > 5000)
				goto searchForBall;
			
            if ( globalBall.x() <(theFieldDimensions.xPosOwnPenaltyArea+xiao)
			&& globalBall.x()>theFieldDimensions.xPosOwnPenaltyArea )
                goto sideKick;
			
			if (theRobotPose.translation.x()>theFieldDimensions.xPosOwnPenaltyArea+da || globalBall.x() > 0)
				goto start;
				
			if(distance>ball.norm())         
				goto defend1;
		}
		action
		{
            LookForward();
			Vector2f tempPos(theFieldDimensions.xPosOwnPenaltyArea+800.f, globalBall.y()*((theFieldDimensions.xPosOwnGroundline-theFieldDimensions.xPosOwnPenaltyArea-800.f) / (theFieldDimensions.xPosOwnGroundline - globalBall.x())));
            WalkToTarget(Pose2f(0.1f, 0.8f, 0.8f), Transformation::fieldToRobot(theRobotPose,tempPos));  //稳不稳有待检测
        }
    }
	
	state(runToBallAndKick)     //踢球状态
	{
		transition
		{
			if(turnInfo!=0)
				goto searchForBall;
			if (state_time > 15000.f)
				goto start;
			if (ball.norm()> 2000.f)
				goto start;
			if(globalBall.x()>0)
				goto start;
			if(std::abs(globalBall.y())>theFieldDimensions.yPosLeftFieldBorder-1000.f)
				goto sideKick;
		}
		action
		{
			if (ball.norm()<300)
			{
				if(theRobotPose.rotation <= 45_deg &&theRobotPose.rotation >= 0_deg) 
				{
                    if(libCodeRelease.between(ball.x(), 150.f, 170.f) 
					&& libCodeRelease.between(ball.y(), 35.f, 55.f)
					||state_time>3000)
					{
                        InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left),Pose2f(ball.angle(),ball.x() - 160.f, ball.y() - 55.f));
					}
					else 
					{
							WalkToTarget(Pose2f(0.2f,1.0f,0.8f),Vector2f(ball.x()-160.f, ball.y()-40.f));  //c1,c2
					}
				}
				
				else if(theRobotPose.rotation <= -45_deg &&theRobotPose.rotation < 0_deg) 
				{
					if(libCodeRelease.between(ball.x(), 150.f, 170.f) 
					&& libCodeRelease.between(ball.y(), -40.f, -20.f)
					||state_time>3000)
					{
                       InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right),Pose2f(ball.angle(),ball));
					}
					else 
					{
						WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(ball.angle(),ball.x()-160.f, ball.y()+30.f)); 
					}
				}
				
				else if(theRobotPose.rotation> 45_deg && theRobotPose.rotation< 180_deg)
				{
									
					if(libCodeRelease.between(ball.x(), 160.f, 180.f) 
					&&libCodeRelease.between(ball.y(), -30.f, -10.f))
					{
						InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 170.f, ball.y() + 12.f));
						turnInfo=2;     //2是左脚 是右半场     然后顺时针转
					}
					else 
						WalkToTarget(Pose2f(0.5f, 1.0f, 1.0f), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x()-170.f, ball.y() + 20.f));
				}
				
				else if (theRobotPose.rotation> -180_deg && theRobotPose.rotation< -45_deg)
				{
					if(libCodeRelease.between(ball.x(), 160.f, 180.f) 
					&&libCodeRelease.between(ball.y(), 10.f, 30.f))
					{
						InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 170.f, ball.y() - 12.f));
						turnInfo=1;    //1是右脚  是左半场   然后逆时针转
					}
					else 
						WalkToTarget(Pose2f(0.5f, 0.8f, 0.8f), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 170.f, ball.y() - 20.f));
				}
			}
			else if(globalBall.x() < theFieldDimensions.xPosOwnPenaltyArea + da)
			{
				WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Vector2f(ball.x()-160.f,ball.y()-40.f));
			}  
		}
	}
	
    state(alignToGoal)
	{
		transition
		{
			if(libCodeRelease.timeSinceBallWasSeen() > 5000)
				goto searchForBall;
				
			if(std::abs(libCodeRelease.angleToGoal) < 10_deg 
			&& std::abs(theBallModel.estimate.position.y()) < 100.f)
				goto alignBehindBall;
				
			if(theRobotPose.translation.x()>theFieldDimensions.xPosOwnPenaltyArea+2500.f 
			|| globalBall.x() > 0)
				goto start;
		}	
		action
		{
			LookForward();
			if(ball.norm()>600)
				WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Transformation::fieldToRobot(theRobotPose,Vector2f (globalBall.x()-450.f, globalBall.y()-45.f)));
			else 
				WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(libCodeRelease.angleToGoal, ball.x() - 160.f, ball.y()-50.f));
		}
	}
  
  
    state(alignBehindBall)     //危险位置，移动到球的后方再行动
	{	
		transition
		{
			if(libCodeRelease.timeSinceBallWasSeen() > 5000)
				goto searchForBall;
				
			if(libCodeRelease.between(ball.y(), 35.f, 55.f) 
			&& libCodeRelease.between(ball.x(), 150.f, 170.f) 
			&& std::abs(libCodeRelease.angleToGoal) < 2_deg)
				goto kick;
    }
		action
		{
			LookForward();
			if(globalBall.x()<theRobotPose.translation.x())
				WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(libCodeRelease.angleToGoal, ball.x(), ball.y() - 60.f));     	//减是让球到45.f位置 45f对应的是左脚
			else
				WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(libCodeRelease.angleToGoal, ball.x() - 160.f, ball.y() - 45.f)); 
		}        
	}
  
	state(kick)
	{
		transition
		{
			if(state_time > 3000 || (state_time > 1000 && action_done))
				goto start;
		}
		action
		{
			LookForward();
			InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(libCodeRelease.angleToGoal, ball.x() - 160.f, ball.y() - 30.f));
		}
	}
  
	state(turnToBall)
	{
		transition
		{
			if(libCodeRelease.timeSinceBallWasSeen() > 2000)
				goto start;
		}
		action
		{
			WalkToTarget(Pose2f(0.5f, 0.f, 0.f), Pose2f(ball.angle(), 0.f, 0.f));
		}
	}	
  
	state(dajiao)
    {
		transition
		{
			if(globalBall.x()>0)
				goto start;
			if(state_time > 3000 || (state_time > 1000 && action_done))
				goto start;
		}
		action
		{
			LookForward();
			if(libCodeRelease.between(ball.x(), 150.f, 170.f)
			&&libCodeRelease.between(ball.y(), 40.f, 60.f)
			||state_time>3000)
				LeftKickForward();
			else
				WalkToTarget(Pose2f(0.5f, 0.8f, 0.8f), Pose2f(libCodeRelease.angleToGoal,ball.x()-160.f, ball.y()-50.f));
		}		
	}
	
	state(sideKick)
	{
		transition
		{
			if(turnInfo!=0)
				goto searchForBall;
				
			if(libCodeRelease.timeSinceBallWasSeen() > 5000)
				goto searchForBall;
				
			if(theRobotPose.translation.x()>theFieldDimensions.xPosOwnPenaltyArea+2500.f 
			|| globalBall.x() > 0)
				goto start;
				
			if(state_time>20000)
				goto start;
		}
		action
		{
			if(theRobotPose.rotation<= 0_deg && theRobotPose.rotation> -180_deg)
			{
				if(libCodeRelease.between(ball.x(), 160.f, 180.f) 
				&&libCodeRelease.between(ball.y(), 10.f, 30.f))
				{
					InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 160.f, ball.y() - 12.f));
					turnInfo=1;
				}
				else
					WalkToTarget(Pose2f(0.2f, 0.6f, 0.5f), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 170.f, ball.y() - 20.f));
			}
			else
			{				
				if(libCodeRelease.between(ball.x(), 160.f, 180.f) 
				&&libCodeRelease.between(ball.y(), -30.f, -10.f))
				{
					InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 170.f, ball.y() + 12.f));
					turnInfo=2;
				}
				else
					WalkToTarget(Pose2f(0.5f, 0.8f, 0.8f), Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x()-170.f, ball.y() + 20.f));	
			}
		}
	}
}

