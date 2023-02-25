// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Team2246;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Drivestation extends SubsystemBase {
  private static GenericHID buttonboardA, buttonboardB;
  private static Joystick lJoy , rJoy;

  /** Creates a new Drivestation. */
  public Drivestation(
    int lJoyPort, 
    int rJoyPort,
    int bbaPort, 
    int bbbPort 
  ) {
    buttonboardA = new GenericHID(bbaPort);
    buttonboardB = new GenericHID(bbbPort);
    lJoy = new Joystick(lJoyPort);
    rJoy = new Joystick(rJoyPort);
  }

  private static double tune(double x){return Math.abs(x)>.0315? x*x*Math.signum(x):0;}

  public BooleanSupplier getBoomLimitOveride(){return ()->buttonboardA.getRawButton(7);}
  
  //ButtonBoard
  public Trigger s00(){return new Trigger(()->buttonboardA.getRawButton(1));}
  public Trigger s01(){return new Trigger(()->buttonboardA.getRawButton(2));}
  public Trigger s02(){return new Trigger(()->buttonboardA.getRawButton(3));}
  public Trigger s03(){return new Trigger(()->buttonboardA.getRawButton(4));}
  public Trigger s10(){return new Trigger(()->buttonboardA.getRawButton(5));}
  public Trigger s11(){return new Trigger(()->buttonboardA.getRawButton(6));}
  public Trigger s12(){return new Trigger(()->buttonboardA.getRawButton(7));}
  public Trigger s13(){return new Trigger(()->buttonboardA.getRawButton(8));}

  public Trigger getHandBrake(){return new Trigger(()->buttonboardA.getRawButton(9));}
 
  public Trigger b00(){return new Trigger(()->buttonboardB.getRawButton( 1));}
  public Trigger b01(){return new Trigger(()->buttonboardB.getRawButton( 2));}
  public Trigger b02(){return new Trigger(()->buttonboardB.getRawButton( 3));}
  public Trigger b03(){return new Trigger(()->buttonboardB.getRawButton( 4));}
  public Trigger b10(){return new Trigger(()->buttonboardB.getRawButton( 5));}
  public Trigger b11(){return new Trigger(()->buttonboardB.getRawButton( 6));}
  public Trigger b12(){return new Trigger(()->buttonboardB.getRawButton( 7));}
  public Trigger b13(){return new Trigger(()->buttonboardB.getRawButton( 8));}
  public Trigger b20(){return new Trigger(()->buttonboardB.getRawButton( 9));}
  public Trigger b21(){return new Trigger(()->buttonboardB.getRawButton( 10));}
  public Trigger b22(){return new Trigger(()->buttonboardB.getRawButton(11));}
  public Trigger b23(){return new Trigger(()->buttonboardB.getRawButton(12));}

  //Left Stick
  public Trigger ls1 (){return new Trigger(()->lJoy.getRawButton(1));}
  public Trigger ls2 (){return new Trigger(()->lJoy.getRawButton(2));}
  public Trigger ls3 (){return new Trigger(()->lJoy.getRawButton(3));}
  public Trigger ls4 (){return new Trigger(()->lJoy.getRawButton(4));}
  public Trigger ls5 (){return new Trigger(()->lJoy.getRawButton(5));}
  public Trigger ls6 (){return new Trigger(()->lJoy.getRawButton(6));}
  public Trigger ls7 (){return new Trigger(()->lJoy.getRawButton(7));}
  public Trigger ls8 (){return new Trigger(()->lJoy.getRawButton(8));}
  public Trigger ls9 (){return new Trigger(()->lJoy.getRawButton(9));}
  public Trigger ls10(){return new Trigger(()->lJoy.getRawButton(10));}
  public Trigger ls11(){return new Trigger(()->lJoy.getRawButton(11));}

  public  double getLeftX(){return tune(lJoy.getX());}
  public  double getLeftY(){return -tune(lJoy.getY());}
  public double getLeftSlider(){return lJoy.getThrottle();}

  //right joystick
  public Trigger rs1 (){return new Trigger(()->rJoy.getRawButton(1));}
  public Trigger rs2 (){return new Trigger(()->rJoy.getRawButton(2));}
  public Trigger rs3 (){return new Trigger(()->rJoy.getRawButton(3));}
  public Trigger rs4 (){return new Trigger(()->rJoy.getRawButton(4));}
  public Trigger rs5 (){return new Trigger(()->rJoy.getRawButton(5));}
  public Trigger rs6 (){return new Trigger(()->rJoy.getRawButton(6));}
  public Trigger rs7 (){return new Trigger(()->rJoy.getRawButton(7));}
  public Trigger rs8 (){return new Trigger(()->rJoy.getRawButton(8));}
  public Trigger rs9 (){return new Trigger(()->rJoy.getRawButton(9));}
  public Trigger rs10(){return new Trigger(()->rJoy.getRawButton(10));}
  public Trigger rs11(){return new Trigger(()->rJoy.getRawButton(11));}
  public Trigger rs12(){return new Trigger(()->rJoy.getRawButton(12));}

  public Trigger rsPOVup   (){return new Trigger(()->getRightPov()==0);}
  public Trigger rsPOVright(){return new Trigger(()->getRightPov()==90);}
  public Trigger rsPOVdown (){return new Trigger(()->getRightPov()==180);}
  public Trigger rsPOVleft (){return new Trigger(()->getRightPov()==270);}

  public double getRightX(){return -tune(rJoy.getRawAxis(0));}
  public double getRightY(){return tune(rJoy.getY());}
  public double getRightZ(){return tune(rJoy.getZ());}
  public double getRightSlider(){return rJoy.getThrottle();}
  public int getRightPov(){return rJoy.getPOV();}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
