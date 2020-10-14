import math
import numpy as np
from scipy.spatial.transform import Rotation as R


class BicycleKinematicOdometry:

    # Memory variables
    # this array save the previous-step encoder position, not the increments!
    oldRotationAngle = [0, 0, 0, 0]
    poseArray = np.array([0, 0, 0])

    def __init__(self):
        self.wheelRadius = 0
        self.Lr = 0
        self.Lf = 0

# wheelRadius,delta_T,Lr,Lf

    def SetWheelRadius(self, wheelRadius):
        self.wheelRadius = wheelRadius

    def SetLr(self, Lr):
        self.Lr = Lr

    def SetLf(self, Lf):
        self.Lf = Lf

    def SetStaticData(self, wheelRadius, Lr, Lf):
        self.SetWheelRadius(wheelRadius)
        self.SetLr(Lr)
        self.SetLf(Lf)

    def ComputeVelocity(self, wheelsRotationAngle, wheelIndex, delta_t):
        '''
        This function allows to compute the wheel linear velocity.
        Parmameters
        ----------
        Return
        ----------
        '''
        # sign compute: note thath it is inverted because of UE4 reverse implementation
        # NOTE: in UE4 negative encoder value rapresents a forward motion and viceversa.
        rotationSign = 1
        if (wheelsRotationAngle <= 0):
            rotationSign = 1
        else:
            rotationSign = -1

        # this rapresent the threeshold after whichi consider encoder saturation. note: this refers to the old encoder value
        hightSaturationThreshoold = 1200
        # this rapresent the threeshold below whichi consider encoder saturation. Note: this refers to the new encoder value
        lowSaturationThreshoold = 200
        # this value save how much the wheel is rotate from the previous step to the actual
        rotationAngleDifference = 0

        # convert into positiv logic: forward is now negative for UE4
        oldRA = self.oldRotationAngle[wheelIndex] * (-1)
        newRA = wheelsRotationAngle * (-1)

        # TODO: I think the following if can better coding with switch case and other structure
        # and several cases can be compress,but for quickest learning i prefer to leave everything as it is.
        # For testing tasks i don't think this should be a computational problem!

        # Forward
        if(oldRA > 0 and newRA > 0):
            # forward without encoder saturation
            if(newRA > oldRA):
                rotationAngleDifference = newRA - oldRA
            # forward with encoder saturation
            if(oldRA > newRA and (oldRA > hightSaturationThreshoold and newRA < lowSaturationThreshoold)):
                rotationAngleDifference = 1800 - (oldRA - newRA)

        # Forward from negative value to positives
        if(oldRA < 0 and newRA > 0):
            rotationSign = 1
            rotationAngleDifference = newRA-oldRA

        # Forward from negative value to smaller negative ones
        if(oldRA < 0 and newRA < 0):  # both negative
            # be sure that i'm not going to saturating the encoder
            if(oldRA > -hightSaturationThreshoold):
                if(oldRA < newRA):  # moving to zero value
                    rotationSign = 1
                    rotationAngleDifference = newRA - oldRA

        # Backward
        if(oldRA < 0 and newRA < 0):
            # backward without encoder saturation
            if(newRA < oldRA):
                rotationAngleDifference = oldRA - newRA
            # backward without encoder saturation
            if(oldRA < -hightSaturationThreshoold and newRA > -lowSaturationThreshoold):
                rotationAngleDifference = 1800 + (oldRA - newRA)

        # Backward from positive value and remaining in positive value
        if(oldRA > 0 and newRA > 0):  # positive values
            # be sure that i'm not going to saturating the encoder
            if(oldRA < hightSaturationThreshoold):
                if(oldRA > newRA):  # backward motion
                    rotationSign = -1
                    rotationAngleDifference = oldRA-newRA

        # Backward from positive value to negative ones
        if(oldRA > 0 and newRA < 0):
            rotationSign = -1
            rotationAngleDifference = oldRA - newRA

        # the wheel circunference
        wheelCircumference = 2*math.pi*self.wheelRadius
        # the velocity of the wheel: space/delta_T
        v = (wheelCircumference * (abs(rotationAngleDifference)/360))/delta_t

        # update the memory variable with new old value
        self.oldRotationAngle[wheelIndex] = wheelsRotationAngle

        return [v, rotationSign]

    def OdometryCar(self, wheelsSteerAngle, wheelsRotationAngle, delta_t):
        # this Odometry model is based on the equations shows in https://www.youtube.com/watch?v=HqNdBiej23I

        # NOTE: this is an assumption!!!
        # We approssimate the steerangle of the car by mean doing
        # the mean of the front-steering-wheels, that are, for our car, the only that changing their direction.
        # according to  http://www.ce.unipr.it/~medici/ctrl.pdf for relative small angle the approssimation is fine
        wheelsSteerAngle = np.mean([wheelsSteerAngle[0], wheelsSteerAngle[1]])

        # vehicle velocity
        # compute the velocity for each wheel
        wheelsRotationAngle = np.asarray(wheelsRotationAngle)

        v0 = self.ComputeVelocity(wheelsRotationAngle[0], 0, delta_t)
        v1 = self.ComputeVelocity(wheelsRotationAngle[1], 1, delta_t)
        v2 = self.ComputeVelocity(wheelsRotationAngle[2], 2, delta_t)
        v3 = self.ComputeVelocity(wheelsRotationAngle[3], 3, delta_t)

        allV = np.array([v0[0], v1[0], v2[0], v3[0]])

        # NOTE: APPORSSIMATION!!
        # we decide to take the mean of the 4 wheel, but it is possibile also to use
        # the slowest or faster 2 wheel, or the fronts ones, or the back ones.
        # It is your choise

        # CODE to use the 2 fasters
        # max1 = np.sort(allV)[len(allV)-1]
        # max2 = np.sort(allV)[len(allV)-2]
        # v = np.mean([max1, max2])

        # Code to use the mean
        v = np.mean(allV)

        # can happen that some wheel became negative before the others
        # and the car is not going backward ( positive logic motion ( see inside ComputeVelocity))
        # so we decide thath the direction of movement of the car change only when all the wheel are rotating
        # in the same towards
        rotationSign = v0[1]
        if (v0[1] == v1[1] == v2[1] == v3[1]):
            rotationSign = v0[1]

        # Compute vehicle orientation angle
        # change steering sign to be compliance with UE4 gps data
        if(rotationSign == 1):
            yawAngle = wheelsSteerAngle * (-1) # this value is in degree
        else:
            yawAngle = wheelsSteerAngle   # this value is in degree

        yawAngle = yawAngle * (math.pi/180)  # convert degree to radians

        # vehicle sideslip Angle  (beta)
        tan = math.tan(yawAngle)
        beta = math.atan((self.Lr/(self.Lr+self.Lr))
                         * tan)  # beta is in radians

        # Compute of the velocity kinematic model

        # first compute the car orientation
        yawRate = (v/(self.Lr*2)) * math.cos(beta) * math.tan(yawAngle)
        newYawAngle = yawRate * delta_t

        # second compute the components of the velocity vector.
        # NOTE: with bycicle model we ar assuming that this vector is positioned
        # at distance Lr from the rear wheel and Lf from the front wheel
        vx = v * math.cos(newYawAngle + beta)
        vy = v * math.sin(newYawAngle + beta)

        # From velocity model to odometric model
        dx = vx * delta_t * rotationSign
        dy = vy * delta_t * rotationSign

        return [dx, dy, newYawAngle]

    def carTrajector(self, wheelsSteerAngle, wheelsRotationAngle, delta_t):
        '''
        This method compute the absolute pose from new data. 
        The fixed Reference system used here is the one that the car have before its first shift
        '''
        # compute the odometry data
        newPoseArray = self.OdometryCar(wheelsSteerAngle, wheelsRotationAngle, delta_t)

        # save data from previous step
        oldPose = np.array(
            [self.poseArray[0], self.poseArray[1], 0]).reshape([1, 3]).T
        old_yaw = self.poseArray[2]

        # We need to compute Roto-Translation from previous pose to the new one

        # Compute Rotation Matrix
        Rotation = R.from_euler('z', old_yaw, degrees=False)
        Rotation_Matrix = Rotation.as_matrix()

        # Compute Translation Matrix by attaching it to Rotation Matrix
        Translation = np.concatenate([Rotation_Matrix, oldPose], axis=1)
        T_r = np.concatenate([Translation, np.array(
            [0, 0, 0, 1]).reshape([1, 4])], axis=0)

        # Prepar odometry new data
        update_pose = np.array(
            [newPoseArray[0], newPoseArray[1], 0, 1]).reshape(1, 4).T

        # Compute the dot product between Roto-Translation Matrix and new Odometry data
        newPose = np.dot(T_r, update_pose).T[:, :-1]

        # Prepare the output
        output = np.array([newPose[0][0], newPose[0]
                           [1], old_yaw+newPoseArray[2]])

        # update the memory variable with new pose just calculate
        self.poseArray = output

        return output
