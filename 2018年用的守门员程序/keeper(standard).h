//实现各种扑倒挡球的功能
option(test2)
{
    const Vector2f ball = theBallModel.estimate.position;   //定义球的相对位置
	const Vector2f globalBall = Transformation::robotToField(theRobotPose, ball);
    static Vector2f tball = Vector2f(0.f,0.f);//队友看到的球的相对位置
	static Vector2f tglobalBall = Vector2f(0.f,0.f);//队友看到球的绝对位置
    std::vector<Teammate> t = theTeammateData.teammates;
	std::vector<Obstacle> o = theObstacleModel.obstacles;
    static int f ;
    Vector2f vBall = theBallModel.estimate.velocity;
	Vector2f vglobalBall = Transformation::robotToField(theRobotPose, vBall);
    //if(o[0].type == Obstacle::someRobot)
    initial_state(start)
	{
		transition
		{
			if (state_time > 3000)
				  goto ballState;
		}
			action
		{
			LookForward();//round
		}

    }
        
    state(ballState)
    {
        transition
        {
            if (libCodeRelease.timeSinceBallWasSeen() < 300)
            {             
                f=1;
                goto judgeState;
            }
            int i = 0;
			for(i=0;i<t.size();i++)
			{
				
                    if (libCodeRelease.timeSinceBallWasSeen() < 300)
                            goto ballState;
                    if(theFrameInfo.getTimeSince(t[i].ball.timeWhenLastSeen)<200)//注意！！：这里就是调用了队友是否看到球的数据的地方，仿照这个形式可以得到相关数据 
				//if(std::isfinite(t[i].ball.estimate.position.x())
                    {    
                        f=0;
                        goto judgeState;
                    }
                
            }
        }
        action
        {
            if(std::abs(theRobotPose.translation.y()) > 50 || std::abs(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline-200) > 50) 
                {
                WalkToTarget(Pose2f(0.2f, 0.6f, 0.5f), Pose2f(libCodeRelease.angleToGoal, theFieldDimensions.xPosOwnGroundline+200-theRobotPose.translation.x() ,0-theRobotPose.translation.y()  ));
                }
				LookRound();
        }
    }
        
    state(judgeState)
    {
        transition
        {
            if (f == 1)
            {
                if ((globalBall.x()<theFieldDimensions.xPosHalfWayLine)||((globalBall.x()>theFieldDimensions.xPosHalfWayLine)&&(vBall.x()<-3.0f))) 
                    goto danger;
                else
                    goto safe;
            }
            if (f == 0)
            {
                if (libCodeRelease.timeSinceBallWasSeen() < 300)
                            goto ballState;
                int i = 0;
                for(i=0;i<t.size();i++)
                {
                    
                        if(theFrameInfo.getTimeSince(t[i].ball.timeWhenLastSeen)<200)
				//if(std::isfinite(t[i].ball.estimate.position.x()))
                        {
                            tglobalBall = Transformation::robotToField(t[i].pose, t[i].ball.estimate.position);
                            tball = Transformation::fieldToRobot(theRobotPose, tglobalBall);
                            if((tglobalBall.x()<theFieldDimensions.xPosHalfWayLine))
                                goto danger;
                            else
                                goto safe;
                        }
                    
                }
            }
        }
        action
        {
            LookForward();
			if(std::abs(theRobotPose.translation.y()) > 50
                || std::abs(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline-200) > 50 ) 
			{
                WalkToTarget(Pose2f(0.2f, 0.6f, 0.5f), Pose2f(libCodeRelease.angleToGoal, theFieldDimensions.xPosOwnGroundline+200-theRobotPose.translation.x() ,0-theRobotPose.translation.y()  ));
			}
            
        }
    
    }
        
    
    state(safe)
    {
        transition
        {
            if (f == 1)
            {
                if ((globalBall.x()<theFieldDimensions.xPosHalfWayLine)||((globalBall.x()>theFieldDimensions.xPosHalfWayLine)&&(vBall.x()<-3.0f))) 
                    goto danger;
                if (libCodeRelease.timeSinceBallWasSeen() > 3000)
                    goto ballState;
            }
            if (f == 0)
            {
                int i = 0;
                for(i=0;i<t.size();i++)
                {
                    
                        if (libCodeRelease.timeSinceBallWasSeen() < 300)
                            goto ballState;
                        if(theFrameInfo.getTimeSince(t[i].ball.timeWhenLastSeen)<200)           
				//if(std::isfinite(t[i].ball.estimate.position.x()))
                        {
                            tglobalBall = Transformation::robotToField(t[i].pose, t[i].ball.estimate.position);
                            tball = Transformation::fieldToRobot(theRobotPose, tglobalBall);
                            if((tglobalBall.x()<theFieldDimensions.xPosHalfWayLine))
                                goto danger;
                        if(theFrameInfo.getTimeSince(t[i].ball.timeWhenLastSeen)>3000)
                            goto ballState;
                        
                        }
                    
                }
            }
                        
        action
        {
            if (f == 1)
			{
				LookAtBall();
				WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(ball.angle(),-4200.f-theRobotPose.translation.x(), 300.f*globalBall.y()/(globalBall.x()+4500.f)-theRobotPose.translation.y())); 
			}
            if (f == 0)
			{   
				LookForward();
                WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(tball.angle(),-4200.f-theRobotPose.translation.x(), 300.f*tglobalBall.y()/(tglobalBall.x()+4500.f)-theRobotPose.translation.y()));
            }
        }
    }
	}
        
    state(danger)
    {
        transition
        {
           if (f == 1)
            {
                if ((globalBall.x()>theFieldDimensions.xPosHalfWayLine)&&(vBall.x()>-3.0f)) 
                    goto safe;
                if (libCodeRelease.timeSinceBallWasSeen() > 3000.f)
                    goto ballState;
                if(globalBall.x() < theFieldDimensions.xPosOwnPenaltyMark+500)
            {
                if(std::abs(globalBall.y()) < theFieldDimensions.yPosLeftPenaltyArea+400)
                {

                    if(vBall.norm() < 5.0f 
                    && globalBall.x() < theFieldDimensions.xPosOwnPenaltyArea+100
                    && std::abs(theRobotPose.translation.y()) < 1100
                    && theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea+100)
                    {
                        goto attack;
                    }
					else if(vBall.x()<-0.7f&&std::abs(theRobotPose.translation.y()-globalBall.y()+(vglobalBall.y()/vglobalBall.x())*(globalBall.x()+4200))<150)
                    goto middelSideDown;
					else if(vBall.x()<-0.7f&&theRobotPose.translation.y()-globalBall.y()+(vglobalBall.y()/vglobalBall.x())*(globalBall.x()+4200)<-150)
                    goto leftSideDown;
					else if(vBall.x()<-0.7f&&theRobotPose.translation.y()-globalBall.y()+(vglobalBall.y()/vglobalBall.x())*(globalBall.x()+4200)>150)
                    goto rightSideDown;
					/*else if(vBall.x() < -1.0f && std::abs(300.f*2/3*globalBall.y()/(globalBall.x()+4500.f*2/3)-theRobotPose.translation.y())<150)
                    {
                        goto middelSideDown;
                    }
					else if(vBall.x() < -1.0f && 300.f*2/3*globalBall.y()/(globalBall.x()+4500.f*2/3)-theRobotPose.translation.y()>150)
					{
						goto leftSideDown;
					}
					else if(vBall.x() < -1.0f && 300.f*2/3*globalBall.y()/(globalBall.x()+4500.f*2/3)-theRobotPose.translation.y()<-150)
					{
						goto rightSideDown;
					}*/
                }
            }
            }
            if (f == 0)
            {
                int i = 0;
                for(i=0;i<t.size();i++)
                {
                    if (libCodeRelease.timeSinceBallWasSeen() < 100)
                            goto ballState;
					
                        
                        if(theFrameInfo.getTimeSince(t[i].ball.timeWhenLastSeen)<200)
				//if(std::isfinite(t[i].ball.estimate.position.x()))
                        {
                            tglobalBall = Transformation::robotToField(t[i].pose, t[i].ball.estimate.position);
                            tball = Transformation::fieldToRobot(theRobotPose, tglobalBall);
                            if((tglobalBall.x()>theFieldDimensions.xPosHalfWayLine))
                                goto safe;
                        }
                        if(theFrameInfo.getTimeSince(t[i].ball.timeWhenLastSeen)>3000)
                            goto ballState;
                    
                }
           
            }
        }
        action
        {
                if (f == 1)
				{
					if ((globalBall.x()+4500.f)/std::abs(globalBall.y())>6.f/11.f)
					{
						LookAtBall();
                        WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(ball.angle(),-4200.f-theRobotPose.translation.x(), 300.f*globalBall.y()/(globalBall.x()+4500.f)-theRobotPose.translation.y())); 
					}
				    else if(globalBall.y()>1100*2/3&&(globalBall.x()+4500.f*2/3)/std::abs(globalBall.y())<6.f/11.f)
					{
						LookAtBall();
                        Vector2f u = Transformation::fieldToRobot(theRobotPose, Vector2f((globalBall.x()+4500.f)*550.f/globalBall.y()-4500.f,550.f));
                        WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(ball.angle(),u.x(),u.y())); 
					}
					else if(globalBall.y()<-1100*2/3)
					{
						LookAtBall();
                        Vector2f u = Transformation::fieldToRobot(theRobotPose, Vector2f((globalBall.x()+4500.f)*-550.f/globalBall.y()-4500.f,-550.f));
                        WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(ball.angle(),u.x(),u.y())); 
					}
                }
				if (f == 0)
				{
					if ((tglobalBall.x()+4500.f)/std::abs(tglobalBall.y())>6.f/11.f)
					{
                        WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(tball.angle(),-4200.f-theRobotPose.translation.x(), 300.f*tglobalBall.y()/(tglobalBall.x()+4500.f)-theRobotPose.translation.y())); 
					}
				    else if(tglobalBall.y()>1100&&(tglobalBall.x()+4500.f*2/3)/std::abs(tglobalBall.y())<6.f/11.f)
					{
						WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(tball.angle(),(tglobalBall.x()+4500.f)*550.f/tglobalBall.y()-4500.f-theRobotPose.translation.x(), 550.f-theRobotPose.translation.y()));
					}
					else if(tglobalBall.y()<-1100)
					{
						WalkToTarget(Pose2f(0.5f,1.0f,1.0f),Pose2f(tball.angle(),(tglobalBall.x()+4500.f)*550.f/tglobalBall.y()-4500.f-theRobotPose.translation.x(), -550.f-theRobotPose.translation.y()));
					}
			    }
		}
	}
	

    

state(attack)
	{
		transition
		{
			if (libCodeRelease.timeSinceBallWasSeen() > theBehaviorParameters.ballNotSeenTimeOut)
			    goto ballState;
            if (globalBall.x() > theFieldDimensions.xPosHalfWayLine)
				goto safe;
            if(globalBall.x() > theFieldDimensions.xPosOwnPenaltyArea+100.f)
                goto judgeState;
            if(std::abs(globalBall.y()) > theFieldDimensions.yPosLeftPenaltyArea)
                goto judgeState;     
					
		}
			action
		{
			if(ball.norm() < 500.f)
			{
			LookForward();
                        
			static int lastballpos=0;
                        
			int theNumberOfObstacle;	
			    theNumberOfObstacle  = theObstacleModel.obstacles.size();	
			int j=0;
			for (int k = 0;k<theNumberOfObstacle;k++)
			{	
				if (theObstacleModel.obstacles[k].center.x()-ball.x()<300 
					&& std::abs(theObstacleModel.obstacles[k].center.y()-ball.y())<300)      //判断前方有障碍
				{
					j++;
					break;
				}							
			}
			if(libCodeRelease.angleToGoal >= -60_deg&& libCodeRelease.angleToGoal <= 60_deg)
			{
			if(j>0)
			{
				if (libCodeRelease.angleToGoal >= -60_deg&& libCodeRelease.angleToGoal <= 0_deg)
				{
					if (libCodeRelease.between(ball.x(), 160.f, 180.f) && libCodeRelease.between(ball.y(), -30.f, -10.f) )
					{
						InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left), 
						Pose2f(libCodeRelease.angleToGoal+10_deg,ball.x() - 170.f, ball.y() + 12.f));
                        //                                                                 150.f, ball.y() + 15.f));
                                
					}
					else 
						WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(libCodeRelease.angleToGoal+10_deg,ball.x() - 170.f, ball.y() + 20.f));
				}
				if (libCodeRelease.angleToGoal > 0_deg&& libCodeRelease.angleToGoal <= 60_deg)
				{
					if (libCodeRelease.between(ball.x(), 160.f, 180.f) && libCodeRelease.between(ball.y(), 10.f, 30.f))
					{
						InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right), Pose2f(libCodeRelease.angleToGoal-10_deg,ball.x() - 170.f, ball.y() - 12.f));
                                
					}
					else
						WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(libCodeRelease.angleToGoal-10_deg,ball.x() - 170.f, ball.y() - 20.f));
				}
			}
			else if(j==0)
			{
				if (libCodeRelease.angleToGoal >= -60_deg&& libCodeRelease.angleToGoal <= 60_deg)
				{
				   if(std::abs(ball.y())>20.f)
				     	lastballpos=0;
				   if(lastballpos==1||ball.y()>0)
				   {
                            //WalkToTarget(Pose2f(10.f, 50*2/3.f, 50*2/3.f), 
                            //              Pose2f(libCodeRelease.angleToGoal,ball.x() - 150.f, ball.y() -20.f));
					   if (libCodeRelease.between(ball.x(), 140.f, 170.f) && libCodeRelease.between(ball.y(), 20.f, 50*2/3.f))
					   {
					     	InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(libCodeRelease.angleToGoal - 8_deg,ball.x() - 160.f, ball.y() - 55.f));
                                //LeftKickForward();                                            160.f, ball.y() - 55.f));
                                //减了15度；
                                lastballpos=0;
					   }
					   else
					   {
						   WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(libCodeRelease.angleToGoal - 8_deg,ball.x() - 150.f, ball.y() -30.f));
												//                                        150.f, ball.y() -30.f)
						   lastballpos=1;
					   }
				   }
				   else if(lastballpos==2 || ball.y()<=0)
				   {
				    	if (libCodeRelease.between(ball.x(), 140.f, 170.f) && libCodeRelease.between(ball.y(), -50*2/3.f, -20.f))
					    {
						   InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(libCodeRelease.angleToGoal + 8_deg,ball.x() - 160.f, ball.y() + 55.f));
                                lastballpos=0;
					    }
					    else
					    {
					      	WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(libCodeRelease.angleToGoal + 8_deg,ball.x() - 150.f, ball.y() +30.f));
						    lastballpos=2;}
					    }
				}
			}
			}
                    else if (libCodeRelease.angleToGoal>60_deg && libCodeRelease.angleToGoal<180_deg)
                    {
                        if (libCodeRelease.between(ball.x(), 160.f, 180.f) && 
                                        libCodeRelease.between(ball.y(), 10.f, 30.f))
                        {
                               InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right), 
                                       Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 170.f, ball.y() - 12.f));
                                
                        }
                        else
                            WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), 
                                            Pose2f(libCodeRelease.angleToGoal-90_deg,ball.x() - 170.f, ball.y() - 20.f));
											//                                          150.f, ball.y() - 20.f));
                    }
                    else if (libCodeRelease.angleToGoal>-180_deg && libCodeRelease.angleToGoal<-60_deg)
                    {
                        if (libCodeRelease.between(ball.x(), 160.f, 180.f) && 
                                    libCodeRelease.between(ball.y(), -30.f, -10.f) )
                            {
                                InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left), 
                                            Pose2f(libCodeRelease.angleToGoal+90_deg,ball.x() - 170.f, ball.y() + 12.f));
                        //                                                                 150.f, ball.y() + 15.f));
                                
                            }
                        else 
                            WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), 
                                            Pose2f(libCodeRelease.angleToGoal+90_deg,ball.x() - 170.f, ball.y() + 20.f));
											//                                                  150.f, ball.y() + 20.f));   
                    }
                }
                else
                    WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(ball.angle(), ball.x(), ball.y()));
		}
	}

    state(leftSideDown)
	{
		transition
		{
			if (state_time > 6000.f) 
				  goto getUp;
		}

			action
		{
		   LookForward();
			//KickForward(KickRequest::leftSideDown);
			SpecialAction(SpecialActionRequest::falldown);
		}
	}
	state(rightSideDown)
	{
		transition
		{
			if (state_time > 6000.f)
				  goto getUp;
		}

			action
		{
			LookForward();
			//KickForward(KickRequest::rightSideDown);
		}
	}
	state(middelSideDown)
	{
		transition
		{
			if (state_time > 2000.f)
				  goto getUp;
		}

			action
		{
		//KickForward(KickRequest::down);
		SpecialAction(SpecialActionRequest::new2);
        LookForward();
        }
	}
	/*state(KgetUp)
	{
		transition
		{
			if (state_time > 6000.f)
				goto KFgetUp;
		}
			action
		{
			GetUp();
		}
	}
	state(KFgetUp)
	{
		transition
		{
			if (state_time > 10000.f)
				goto judgeState;
            //if (libCodeRelease.timeSinceBallWasSeen() < 300)
			//goto judgeState;
		}
			action
		{
WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
		}
	}   */ 
	state(getUp)
	{
		transition
		{
			if (state_time > 3000.f)
				goto judgeState;
		}
			action
		{
			GetUp();
		}
	}

	state(searchForBall)
	{
		transition
		{
		  if (libCodeRelease.timeSinceBallWasSeen() < 300)
			goto ballState;
            
		}
			action
		{

        /*    if(state_time % 3 == 0)
                SetHeadPanTilt(1.f, 0.38f, 1.5f);
            else if(state_time % 3 == 1)
                SetHeadPanTilt(0.f, 0.38f, 1.5f);
            else if(state_time % 3 == 2)
                SetHeadPanTilt(1.f, 0.38f, 1.5f);
          */
          if (std::abs(theRobotPose.translation.y()) > 50
				|| std::abs(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline) > 50 ) {
				WalkToTarget(Pose2f(0.2f, 0.6f, 0.5f), Pose2f(libCodeRelease.angleToGoal, theFieldDimensions.xPosOwnGroundline+100-theRobotPose.translation.x() ,0-theRobotPose.translation.y()  ));
		  }
            LookForward();//round0.5
            //else 
              //  LookForward();
		}
	}
}
      
        
   
