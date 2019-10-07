//负责左半场的防守
option(defender1)
{
    const Vector2f ball = theBallModel.estimate.position; //球相对坐标
    const Vector2f globalBall = Transformation::robotToField(theRobotPose, ball);//球的绝对位置
    const float a=globalBall.x();
    const float b=globalBall.y();
    const float c=theRobotPose.translation.x(); //机器人绝对坐标
    const float d=theRobotPose.translation.y();
    const Vector2f initial=Transformation::fieldToRobot(theRobotPose,Vector2f(-3400.f,600.f));//走到初始位置
    const Vector2f qiumen=Transformation::fieldToRobot(theRobotPose,Vector2f(-4500.f,0.f));//球门的坐标x:theFieldDimensions.xPosOwnGroundline y:theFieldDimensions.yPosCenterGoal
    const float theta1=atan(-(qiumen.x()-theBallModel.estimate.position.x())/(qiumen.y()-theBallModel.estimate.position.y()));
    const float chuix = ball.norm()*cos(theta1)*cos(theBallModel.estimate.position.angle()-theta1);
   const float chuiy = ball.norm()*sin(theta1)*cos(theBallModel.estimate.position.angle()-theta1);//dangqiu垂足角度
   const Vector2f chuizu=Transformation::robotToField(theRobotPose,Vector2f(chuix,chuiy));//垂足转化成绝对坐标
   const Vector2f back=Transformation::fieldToRobot(theRobotPose,Vector2f(-700.f,600.f));//到后场walktoback
   const Vector2f line2=Transformation::fieldToRobot(theRobotPose,Vector2f(c,b));//turntoline2的位置
   const float qiex=-3900.f;
   const float qiey=b-((a+3900.f)*b/(a+4500.f));
   const Vector2f qie=Transformation::fieldToRobot(theRobotPose,Vector2f(qiex,qiey));//切点的相对位置
   initial_state(start1)//10s内不能进攻
   {
       transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto start2;
      if(state_time>1000)
        goto start;
    }
    action
    {
      Stand();
      LookAtBall();
    }
  }
  
  state(start2)//原地找球
  {
      transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen<300)
        goto start1;
    }
    action
    {
      Stand();
      LookAround();
    }
  }
  
  
  
  state(start)
  {
      transition
    {
      if(state_time > 500)
        goto searchForBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
  }
  
 /* initial_state(start)//调试跳过10s内不能动
  {
      transition
    {
      if(state_time > 500)
        goto searchForBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
  }*/
  
  state(searchForBall)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen <300)
          goto panduanField;
      }
      action
      {
          LookAround();
          WalkAtRelativeSpeed(Pose2f(0.7, 0.f, 0.f));
      }
      
  }
  
  state(turnToBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
        goto walkToBall;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }
  
/*  state(walkToBall)
  {
      transition
      {
         if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
         goto searchForBall;
         if(theBallModel.estimate.position.norm() < 500.f)
         goto alignToTarget;
      }
      action
      {
          LookAtBall();
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);//50.f50.f50.f
          
      }
  }*/
  
  state(alignToTarget)
  {
       transition
    {
        if(theLibCodeRelease.timeSinceBallWasSeen > 5000)
            goto searchForBall;
        if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         )
            goto kick;
    }
    action
    {
      LookAtBall();
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
    }
  }
  
  state(alignToGoal)
  {
      transition
    {
        if(theLibCodeRelease.timeSinceBallWasSeen > 5000)
            goto searchForBall;
        if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
            goto alignBehindBall;
    }
    action
    {
      LookAtBall();
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }
  
  state(alignBehindBall)
  {
      transition
    {
        if(theLibCodeRelease.timeSinceBallWasSeen > 5000)
            goto searchForBall;
        if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg)
            goto kick;
    }
    action
    {
      LookAtBall();
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
    }
  }
  
  state(kick)//需要改成大脚
  {
      transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
    }
  }
  
  state(panduanField)//判断球在前半场还是后半场
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen >4000)
          goto searchForBall;
          if(theLibCodeRelease.timeSinceBallWasSeen<300)
          {
              if(globalBall.x()>0)//球在前半场
              {
                  if(globalBall.x()>800)
                      goto walkToInitial;//球在较前场则走到初始站位
                  else goto turnToBall;//球在中间缓冲区
              }
              else goto panduanBall;//球在后半场判断球的位置
          }    
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          Stand(); 
      }
  }
  
  state(walkToInitial)//走到初始位置
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen >10000)
          goto searchForBall;
          if(initial.norm()<100)
          goto turnToBall;
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          WalkToTarget(Pose2f(80.f,80.f,80.f),Pose2f(theLibCodeRelease.angleToGoal,initial.x(),initial.y()));
          
      }
  }
  
  state(panduanBall)//球在后场判断球位置，else沿用单后卫的panduan
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen > 5000)
          goto searchForBall;
          if(theLibCodeRelease.timeSinceBallWasSeen<300)
          {
              if(globalBall.y()<0)//球在左半场
              goto panduanDefender2;//如果defender2离球较defender2近，则过去抢球
              else 
             {
                 if((a<-3900&&b<theFieldDimensions.yPosLeftPenaltyArea)&&(b>theFieldDimensions.yPosRightPenaltyArea))//球在禁区
                 goto turnToBall;
                 if(a<-3900&&((b>theFieldDimensions.yPosLeftPenaltyArea)||(b<theFieldDimensions.yPosRightPenaltyArea)))
                 goto panduanren;//判断是否在禁区的一侧
                 else goto panduanqiu;//走到跟球一条线挡住敌人
              }
          }
              
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          Stand();
      }
  }
  
   state(panduanqiu)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
         if(c>a)
             goto walkToBall;//turnToRaoqiuhou;
        if(c<a)
             goto panduanyuanjin;
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
      }
  }
  

  state(walkToBall)
  {
      transition
      {
          if (c>150)
              goto panduanBack;
        if((c<-3900&&d<theFieldDimensions.yPosLeftPenaltyArea)&&(d>theFieldDimensions.yPosRightPenaltyArea))//球在禁区
           goto walkToBall;
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
          //if(theBallModel.estimate.position.norm() < 500.f)
              //goto alignToGoal;
         if(std::abs(theBallModel.estimate.position.angle()) > 10_deg)
             goto turnToBall;
        if(ball.norm() < 500.f&&(c-a)<0)//theBallModel.estimate.position.norm() 
          goto alignToTarget;
		   
		if(ball.norm() < 250.f&&(c-a)>0)
		  goto raoqiu;
      }
      action
      {
          LookAtBall();
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);//50.f50.f50.f
          
      }
  }
    
  state(panduanyuanjin)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
          else if(ball.norm()>700.f)
              goto panduanline;
              else goto turnToBall;
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
      }
  }
  
  state(dangqiu)//走到球与球门连线的垂直距离上
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
              if((std::abs(chuiy) > 200))
                goto turnToLine;
          if(sqrt(chuix*chuix+chuiy*chuiy)<100)
              goto turnToBall;
      }
      action
      {
        LookAtBall();
		WalkToTarget(Pose2f(60.f,60.f,60.f),Pose2f(theta1,chuix,chuiy));
           
      }
  }
  
  
  state(turnToLine)//球在球门两边就走到垂足
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
            if(std::abs(chuiy) < 50)
                goto dangqiu;
      }
      action
      {
          LookAtBall();
          //WalkToTarget(Pose2f(80.f,80.f,80.f),Pose2f(theta1,0.f,0.f));
         if(chuiy<-100)
          WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
          if(chuiy>100)
              WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
      }
  }
  
  state(turnToBack)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>10000)
              goto searchForBall;
          if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg)
              goto walkToBack;
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          if(theLibCodeRelease.angleToGoal>10_deg)
               WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
           if(theLibCodeRelease.angleToGoal<-10_deg)
               WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
      }
  }
  
  state(walkToBack)//不过半场
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>8000||back.norm()<100)
              goto searchForBall;
          if(std::abs(back.angle()) > 10_deg)
              goto turnToBack;

              
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          WalkToTarget(Pose2f(50.f,80.f,80.f),Pose2f(theLibCodeRelease.angleToGoal,back.x(),back.y()));
      }
  }
  
 
   state(raoqiu)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg)
        goto alignBehindBall;
        
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
	  if(b<theFieldDimensions.yPosCenterGoal)
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(ball.angle(), theBallModel.estimate.position.x() - 220.f, -100.f));
      if(b>=theFieldDimensions.yPosCenterGoal)
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(ball.angle(), theBallModel.estimate.position.x() - 220.f, 100.f));
	  
    }
  }
  
  state(search2)//站着不动找球
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
            if(theLibCodeRelease.timeSinceBallWasSeen<300)
                goto panduanBall;
      }
      action
      {
          LookAround();
          Stand();
      }
  }
  
  state(panduanline)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
          /*if((d<theFieldDimensions.yPosRightGoal&&c<-3900)||(d>theFieldDimensions.yPosLeftGoal&&c<-3900))//如果人在禁区两边，则挡球的时候会出现走进禁区的情况
              goto turnToJinqu;
          else  
          {*/
            if(b<theFieldDimensions.yPosLeftGoal-150&&b>theFieldDimensions.yPosRightGoal+150)//方向注意
                goto turnToLine2;//走到和球y坐标相同
            if(b>theFieldDimensions.yPosLeftGoal-150||b<theFieldDimensions.yPosRightGoal+150)
                goto panduanqiedian;//走到垂足
           
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          Stand();
      }
  }
  
  state(turnToLine2)//球在球门方向以内，走到和球一条线上
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>10000)
              goto searchForBall;
            if(std::abs(line2.y()) < 50)
                goto dangqiu2;
      }
      action
      {
          LookAtBall();
         // WalkToTarget(Pose2f(80.f,80.f,80.f),Pose2f(line2.angle(),0.f,0.f));
         //WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
          if(line2.y()<-100)
          WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
          if(line2.y()>100)
              WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
      }
      
  }

  state(dangqiu2)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000&&state_time>10000)
              goto searchForBall;
              if(std::abs(line2.y()) > 200)
                goto turnToLine2;
          if(sqrt(line2.x()*line2.x()+line2.y()*line2.y())<300)
              goto turnToBall;
      }
      action
      {
        LookAtBall();
		WalkToTarget(Pose2f(60.f,60.f,60.f),Pose2f(line2.angle(),line2.x(),line2.y()));
           
      }
  }
  
  state(panduanqiedian)//挡球的时候判断垂足的位置
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
          if(chuizu.y()<theFieldDimensions.yPosLeftPenaltyArea && chuizu.y()>theFieldDimensions.yPosRightPenaltyArea
            && chuizu.x()<theFieldDimensions.xPosOwnPenaltyArea)
                goto turnToQiedian;
            else goto turnToLine;
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          Stand();
      }
  }
  
  state(turnToQiedian)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
            if(std::abs(qie.angle()) < 10_deg)
                goto walkToQiedian;
      
      }
      action
      {
          LookAtBall();
          if(qie.y()<-100)
             WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
          if(qie.y()>100)
              WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
      }
  }
  
  state(walkToQiedian)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
              if((std::abs(qie.angle()) > 15_deg))
                goto turnToQiedian;
          if(sqrt(qie.x()*qie.x()+qie.y()*qie.y())<100)
              goto turnToBall;
      }
      action
      {
        LookAtBall();
		WalkToTarget(Pose2f(60.f,60.f,60.f),Pose2f(qie.angle(),qie.x(),qie.y()));
           
      }
  }
  
  state(panduanren)//球在禁区判断人球是否同一个地方
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
          if((d>theFieldDimensions.yPosLeftPenaltyArea && b>theFieldDimensions.yPosLeftPenaltyArea && a<-3900.f&& c<-3900.f)||(a<-3900.f&&c<-3900.f&&d<theFieldDimensions.yPosRightPenaltyArea && b<theFieldDimensions.yPosRightPenaltyArea))
              goto turnToBall;
          if((d>theFieldDimensions.yPosLeftPenaltyArea && b<theFieldDimensions.yPosRightPenaltyArea &&a<-3900.f&& c<-3900.f)||(a<-3900.f&& c<-3900.f&&d<theFieldDimensions.yPosRightPenaltyArea && b>theFieldDimensions.yPosLeftPenaltyArea))
              goto walkToInitial;
              else goto turnToBall;
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          Stand();
      }
  }
//需要改动状态家，加通讯
  state(panduanDefender2)//如果defender2离球较defender1近，则过去抢球
  {
      transition
      {
           if(theLibCodeRelease.timeSinceBallWasSeen>5000)
              goto searchForBall;
          if(ball.norm()<300)
              goto turnToBall;
          if(ball.norm()>300||ball.norm()==300)
              goto walkToInitial;
          
      }
      action
      {
          HeadControlMode(HeadControl::lookForward);
          Stand();
      }
  
  }
  
   state(panduanBack)//从walkToBall跳进去的
  {
      transition
    {
        if(c>100)//不过半场
            goto turnToBack;
                else
            goto searchForBall;
            
       
    }
       
      action
    {
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
  }
}
