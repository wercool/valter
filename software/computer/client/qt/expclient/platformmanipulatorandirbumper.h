#ifndef PLATFORMMANIPULATORANDIRBUMPER_H
#define PLATFORMMANIPULATORANDIRBUMPER_H

#include <string>
#include <map>

#include "tasks/itask.h"
#include "ivaltermodule.h"

using namespace std;

class PlatformManipulatorAndIRBumper: public IValterModule
{
public:
    static PlatformManipulatorAndIRBumper *getInstance();

    static string getControlDeviceId();

    void stopAll();
    void resetToDefault();
    void loadDefaults();
    void setModuleInitialState();
    void spawnProcessMessagesQueueWorkerThread();
    void initTcpInterface();
    void initTcpCommandAcceptorInterface();
    void processControlDeviceResponse(string response);
    unsigned int executeTask(std::string taskScriptLine);

    bool getPower24VOnOff() const;
    void setPower24VOnOff(bool value);
    void setPower24VOn();
    void setPower24VOff();

    bool prepareManLink1Movement();
    bool prepareManLink2Movement();

    //manipulator
    const static double rootX;
    const static double rootY;
    const static double man_l1;
    const static double man_l2;
    const static double man_l1_l2;
    const static double man_l2_l3;
    const static double man_l3;
    static double man_a;
    static double man_b;
    static double man_g;

    //------------------------------------------------man link1 drive
    bool getLink1MovementDirection() const;
    bool setLink1MovementDirection(bool value);

    bool getLink1MotorActivated() const;
    void setLink1MotorActivated(bool value);

    bool getLink1MotorStop() const;
    void setLink1MotorStop(bool value);

    int getLink1MotorDuty() const;
    void setLink1MotorDuty(int value);

    int getLink1MotorDutyMax() const;
    void setLink1MotorDutyMax(int value);

    int getLink1MotorAcceleration() const;
    void setLink1MotorAcceleration(int value);

    int getLink1MotorDeceleration() const;
    void setLink1MotorDeceleration(int value);

    int getLink1MotorDutyPresetCur() const;
    void setLink1MotorDutyPresetCur(int value);

    int getLink1MotorDutyPresetMin() const;
    void setLink1MotorDutyPresetMin(int value);

    int getLink1MotorDutyPresetMax() const;
    void setLink1MotorDutyPresetMax(int value);

    //------------------------------------------------man link2 drive
    int getLink2MotorDuty() const;
    void setLink2MotorDuty(int value);

    int getLink2MotorDutyMax() const;
    void setLink2MotorDutyMax(int value);

    int getLink2MotorAcceleration() const;
    void setLink2MotorAcceleration(int value);

    int getLink2MotorDeceleration() const;
    void setLink2MotorDeceleration(int value);

    bool getLink2MovementDirection() const;
    bool setLink2MovementDirection(bool value);

    bool getLink2MotorActivated() const;
    void setLink2MotorActivated(bool value);

    bool getLink2MotorStop() const;
    void setLink2MotorStop(bool value);

    int getLink2MotorDutyPresetCur() const;
    void setLink2MotorDutyPresetCur(int value);

    int getLink2MotorDutyPresetMin() const;
    void setLink2MotorDutyPresetMin(int value);

    int getLink2MotorDutyPresetMax() const;
    void setLink2MotorDutyPresetMax(int value);

    bool getLink1MotorAccelerating() const;
    void setLink1MotorAccelerating(bool value);

    bool getLink1MotorDecelerating() const;
    void setLink1MotorDecelerating(bool value);

    bool getLink2MotorAccelerating() const;
    void setLink2MotorAccelerating(bool value);

    bool getLink2MotorDecelerating() const;
    void setLink2MotorDecelerating(bool value);

    //------------------------------------------------man tilt
    void manLink3TiltUp();
    void manLink3TiltDown();
    void manLink3Stop();

    //------------------------------------------------man gripper open/close
    void manGripperOpen();
    void manGripperClose();
    void manGripperStop();

        //------------------------------------------------man gripper rotation
    int getManGripperRotationMotorDuty() const;
    void setManGripperRotationMotorDuty(int value);
    void manGripperRotateCW();
    void manGripperRotateCCW();
    void manGripperRotateStop();

    //-------------------------------------------------manipulator readings triggers
    /* triggers */
                    bool getLink1PositionTrack() const;
                    void setLink1PositionTrack(bool value);

                    bool getLink1PositionADC() const;
                    void setLink1PositionADC(bool value);

                    bool getLink2PositionTrack() const;
                    void setLink2PositionTrack(bool value);

                    bool getLink2PositionADC() const;
                    void setLink2PositionADC(bool value);

                    bool getLink1CurrentTrack() const;
                    void setLink1CurrentTrack(bool value);

                    bool getLink1CurrentADC() const;
                    void setLink1CurrentADC(bool value);

                    bool getLink2CurrentTrack() const;
                    void setLink2CurrentTrack(bool value);

                    bool getLink2CurrentADC() const;
                    void setLink2CurrentADC(bool value);

                    double getLink1Position() const;
                    void setLink1Position(double value);

                    double getLink2Position() const;
                    void setLink2Position(double value);

                    double getLink1Current() const;
                    void setLink1Current(double value);

                    double getLink2Current() const;
                    void setLink2Current(double value);

                    bool getGripperTiltTrack() const;
                    void setGripperTiltTrack(bool value);

                    bool getGripperTiltADC() const;
                    void setGripperTiltADC(bool value);

                    bool getGripperRotationTrack() const;
                    void setGripperRotationTrack(bool value);

                    bool getGripperRotationADC() const;
                    void setGripperRotationADC(bool value);

                    bool getGripperPositionTrack() const;
                    void setGripperPositionTrack(bool value);

                    bool getGripperPositionADC() const;
                    void setGripperPositionADC(bool value);

    int getLink1ADCPosition() const;
    void setLink1ADCPosition(int value);

    int getLink2ADCPosition() const;
    void setLink2ADCPosition(int value);

    int getLink1ADCCurrent() const;
    void setLink1ADCCurrent(int value);

    int getLink2ADCCurrent() const;
    void setLink2ADCCurrent(int value);

    int getGripperADCTilt() const;
    void setGripperADCTilt(int value);

    double getGripperTilt() const;
    void setGripperTilt(double value);

    int getGripperADCRotation() const;
    void setGripperADCRotation(int value);

    double getGripperRotation() const;
    void setGripperRotation(double value);

    int getGripperADCPosition() const;
    void setGripperADCPosition(int value);

    double getGripperPosition() const;
    void setGripperPosition(double value);

    void setIRBumperTrack(int idx, bool state);
    bool getIRBumperTrack(int idx);
    bool getIRBumperTracked();

    void setIRBumperTicks(int idx, bool state);
    bool getIRBumperTicks(int idx);

    bool getIrBumperInitialized() const;
    void setIrBumperInitialized(bool value);

    bool getIrBumperEnabled() const;
    void setIrBumperEnabled(bool value);

    int getIrBumperFrequency() const;
    void setIrBumperFrequency(int value);

    int getIrBumperDuty() const;
    void setIrBumperDuty(int value);

    void setIrBumperTicksReading(int idx, int value);
    int getIrBumperTicksReading(int idx);

    bool getGripperForceSensor1Track() const;
    void setGripperForceSensor1Track(bool value);

    bool getGripperForceSensor2Track() const;
    void setGripperForceSensor2Track(bool value);

    bool getGripperForceSensor3Track() const;
    void setGripperForceSensor3Track(bool value);

    bool getGripperObjectDetectorTrack() const;
    void setGripperObjectDetectorTrack(bool value);

    bool getGripperTiltMotorCurrentTrack() const;
    void setGripperTiltMotorCurrentTrack(bool value);

    bool getGripperOpenCloseMotorCurrentTrack() const;
    void setGripperOpenCloseMotorCurrentTrack(bool value);

    bool getGripperRotationMotorCurrentTrack() const;
    void setGripperRotationMotorCurrentTrack(bool value);

    bool getGripperForceSensor1ADC() const;
    void setGripperForceSensor1ADC(bool value);

    bool getGripperForceSensor2ADC() const;
    void setGripperForceSensor2ADC(bool value);

    bool getGripperForceSensor3ADC() const;
    void setGripperForceSensor3ADC(bool value);

    bool getGripperObjectDetectorADC() const;
    void setGripperObjectDetectorADC(bool value);

    bool getGripperTiltMotorCurrentADC() const;
    void setGripperTiltMotorCurrentADC(bool value);

    bool getGripperOpenCloseMotorCurrentADC() const;
    void setGripperOpenCloseMotorCurrentADC(bool value);

    bool getGripperRotationMotorCurrentADC() const;
    void setGripperRotationMotorCurrentADC(bool value);

    int getGripperForceSensor1ADCValue() const;
    void setGripperForceSensor1ADCValue(int value);

    int getGripperForceSensor2ADCValue() const;
    void setGripperForceSensor2ADCValue(int value);

    int getGripperForceSensor3ADCValue() const;
    void setGripperForceSensor3ADCValue(int value);

    int getGripperObjectDetectorADCValue() const;
    void setGripperObjectDetectorADCValue(int value);

    int getGripperTiltMotorCurrentADCValue() const;
    void setGripperTiltMotorCurrentADCValue(int value);

    int getGripperOpenCloseMotorCurrentADCValue() const;
    void setGripperOpenCloseMotorCurrentADCValue(int value);

    int getGripperRotationMotorCurrentADCValue() const;
    void setGripperRotationMotorCurrentADCValue(int value);

private:
    PlatformManipulatorAndIRBumper();
    static PlatformManipulatorAndIRBumper* pPlatformManipulatorAndIRBumper;         // PLATFORM-MANIPULATOR-AND-IR-BUMPER's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();
    void manLink1MovementWorker();
    void manLink2MovementWorker();
    void manipulatorReadingsWorker();
    void irBumperReadingWorker();

    bool power24VOnOff;

    //------------------------------------------------man link1
    int link1MotorDuty;
    int link1MotorDutyMax;
    int link1MotorAcceleration;
    int link1MotorDeceleration;

    bool link1MotorAccelerating;
    bool link1MotorDecelerating;

    bool link1MovementDirection;
    bool link1MotorActivated;
    bool link1MotorStop;

    //presets
    int link1MotorDutyPresetCur;
    int link1MotorDutyPresetMin;
    int link1MotorDutyPresetMax;

    //------------------------------------------------man link2
    int link2MotorDuty;
    int link2MotorDutyMax;
    int link2MotorAcceleration;
    int link2MotorDeceleration;

    bool link2MotorAccelerating;
    bool link2MotorDecelerating;

    bool link2MovementDirection;
    bool link2MotorActivated;
    bool link2MotorStop;

    //presets
    int link2MotorDutyPresetCur;
    int link2MotorDutyPresetMin;
    int link2MotorDutyPresetMax;

    //------------------------------------------------man gripper rotation
    int manGripperRotationMotorDuty;


    //-------------------------------------------------manipulator readings
    /*************** triggers ************************/
    bool link1PositionTrack;
    bool link2PositionTrack;
    bool link1CurrentTrack;
    bool link2CurrentTrack;
    bool gripperTiltTrack;
    bool gripperRotationTrack;
    bool gripperPositionTrack;

    bool gripperForceSensor1Track;
    bool gripperForceSensor2Track;
    bool gripperForceSensor3Track;
    bool gripperObjectDetectorTrack;
    bool gripperTiltMotorCurrentTrack;
    bool gripperOpenCloseMotorCurrentTrack;
    bool gripperRotationMotorCurrentTrack;

    bool link1PositionADC;
    bool link2PositionADC;
    bool link1CurrentADC;
    bool link2CurrentADC;
    bool gripperTiltADC;
    bool gripperRotationADC;
    bool gripperPositionADC;

    bool gripperForceSensor1ADC;
    bool gripperForceSensor2ADC;
    bool gripperForceSensor3ADC;
    bool gripperObjectDetectorADC;
    bool gripperTiltMotorCurrentADC;
    bool gripperOpenCloseMotorCurrentADC;
    bool gripperRotationMotorCurrentADC;

    /*************** readings ************************/
    int link1ADCPosition;
    int link2ADCPosition;
    int link1ADCCurrent;
    int link2ADCCurrent;
    int gripperADCTilt;
    int gripperADCRotation;
    int gripperADCPosition;

    int gripperForceSensor1ADCValue;
    int gripperForceSensor2ADCValue;
    int gripperForceSensor3ADCValue;
    int gripperObjectDetectorADCValue;
    int gripperTiltMotorCurrentADCValue;
    int gripperOpenCloseMotorCurrentADCValue;
    int gripperRotationMotorCurrentADCValue;

    double link1Position;
    double link2Position;
    double link1Current;
    double link2Current;
    double gripperTilt;
    double gripperRotation;
    double gripperPosition;

    //IR Bumper
    bool irBumperTrack[16];
    bool irBumperTicks[16];

    int irBumperTicksReading[16];

    bool irBumperInitialized;
    bool irBumperEnabled;

    int irBumperFrequency;
    int irBumperDuty;

    /**************************************************** TASKS ******************************************************/
    std::map<std::string, function<ITask*(void)>> tasks;
};

#endif // PLATFORMMANIPULATORANDIRBUMPER_H
