import QtQuick 2.0
import unmansol.erp42.pcanmanager 0.1

PCanConfPageForm {
    //FIXIT: Bidirectional binding.
    onActiveCANChanged: {
        pcanManager.Active = activeCAN
    }
    onAutoControlChanged: {
        pcanManager.AutoEnable = autoControl
    }
    onEstopControlChanged: {
        pcanManager.EstopEnable = estopControl
    }
    onSteerControlChanged: {
        pcanManager.SteerEnable = steerControl
    }
    onSpeedControlChanged: {
        pcanManager.SpeedEnable = speedControl
    }
    onBrakeControlChanged: {
        pcanManager.BrakeEnable = brakeControl
    }

    onGearDriveChanged: {
        pcanManager.GearDrive = gearDrive
    }
    onGearNeutralChanged: {
        pcanManager.GearNeutral = gearNeutral
    }
    onGearReverseChanged: {
        pcanManager.GearReverse = gearReverse
    }

    onSteerAngleChanged: {
        pcanManager.SteerAngle = steerAngle
    }
    onSpeedChanged: {
        pcanManager.Speed = speed
    }
    onBrakeChanged: {
        pcanManager.Brake = brake
    }

    onCycleChanged: {
        pcanManager.Cycle = cycle
    }

}
