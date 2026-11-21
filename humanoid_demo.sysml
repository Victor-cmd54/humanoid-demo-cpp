package HumanoidDemo {
    import ScalarValues::*;

    enum def GaitPhase {
        enum stand;
        enum walk;
        enum run;
        enum jump;
        enum navigate;
    }

    port def MotorCommandPort {
        attribute torque : Real;
    }

    port def CommunicationPort {
        attribute payload : ScalarValues::Integer;
    }

    part def MotorBase {
        attribute name : String;
        attribute ratedTorqueNm : Real;
        port command : MotorCommandPort;
    }

    part def LegMotor specializes MotorBase;
    part def ArmMotor specializes MotorBase;

    part def CommunicationBase {
        abstract;
        port transmitPort : CommunicationPort;
    }

    part def CANFDInterface specializes CommunicationBase;
    part def EtherCATInterface specializes CommunicationBase;

    part def Ros2Node {
        attribute nodeName : String;
    }

    part def MicroRosClient {
        attribute agentName : String;
    }

    part def HumanoidController {
        attribute phase : GaitPhase;

        part leftHip : LegMotor;
        part rightHip : LegMotor;
        part leftKnee : LegMotor;
        part rightKnee : LegMotor;
        part leftShoulder : ArmMotor;
        part rightShoulder : ArmMotor;

        part canfd : CANFDInterface;
        part ethercat : EtherCATInterface;
        part ros2 : Ros2Node;
        part microros : MicroRosClient;

        action def SetPhase {
            in phaseInput : GaitPhase;
        }

        action def ExecuteControlLoop {
            action readPhase;
            action commandHips;
            action transmitCANFD;
            action transmitEtherCAT;
            action publishGaitPhase;
            action publishStatus;

            succession readPhase then commandHips;
            succession commandHips then transmitCANFD;
            succession transmitCANFD then transmitEtherCAT;
            succession transmitEtherCAT then publishGaitPhase;
            succession publishGaitPhase then publishStatus;
        }

        perform ExecuteControlLoop;
    }
}
