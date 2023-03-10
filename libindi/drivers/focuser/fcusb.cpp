/*******************************************************************************
 Copyright(c) 2019 Jasem Mutlaq. All rights reserved.

 Shoestring FCUSB Focuser

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include "fcusb.h"

#include <cmath>
#include <cstring>
#include <memory>

#define FOCUS_SETTINGS_TAB "Settings"

static std::unique_ptr<FCUSB> fcusb(new FCUSB());

void ISGetProperties(const char *dev)
{
    fcusb->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    fcusb->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    fcusb->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    fcusb->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle *root)
{
    fcusb->ISSnoopDevice(root);
}

FCUSB::FCUSB()
{
    setVersion(0, 2);

    FI::SetCapability(FOCUSER_HAS_VARIABLE_SPEED | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
    setSupportedConnections(CONNECTION_NONE);
}

bool FCUSB::Connect()
{
    if (isSimulation())
    {
        SetTimer(POLLMS);
        return true;
    }

    handle = hid_open(0x134A, 0x9023, nullptr);

    if (handle == nullptr)
    {
        LOG_ERROR("No FCUSB focuser found.");
        return false;
    }
    else
    {
        SetTimer(POLLMS);
    }

    return (handle != nullptr);
}

bool FCUSB::Disconnect()
{
    if (isSimulation() == false)
    {
        hid_close(handle);
        hid_exit();
    }

    return true;
}

const char *FCUSB::getDefaultName()
{
    return "FCUSB";
}

bool FCUSB::initProperties()
{
    INDI::Focuser::initProperties();

    FocusSpeedN[0].min = 0;
    FocusSpeedN[0].max = 255;

    // PWM Scaler
    IUFillSwitch(&PWMScalerS[0], "PWM_1_1", "1:1", ISS_ON);
    IUFillSwitch(&PWMScalerS[1], "PWM_1_4", "1:4", ISS_OFF);
    IUFillSwitch(&PWMScalerS[2], "PWM_1_16", "1:6", ISS_OFF);
    IUFillSwitchVector(&PWMScalerSP, PWMScalerS, 3, getDeviceName(), "PWM_SCALER", "PWM Scale", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    addSimulationControl();

    return true;
}

bool FCUSB::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineSwitch(&PWMScalerSP);
    }
    else
    {
        deleteProperty(PWMScalerSP.name);
    }

    return true;
}

void FCUSB::TimerHit()
{
    if (!isConnected())
        return;


    SetTimer(POLLMS);
}

bool FCUSB::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Focus Step Mode
        if (strcmp(PWMScalerSP.name, name) == 0)
        {
            IUUpdateSwitch(&PWMScalerSP, states, names, n);

            pwmStatus = static_cast<PWMBits>(IUFindOnSwitchIndex(&PWMScalerSP));

            PWMScalerSP.s = setStatus() ? IPS_OK : IPS_ALERT;

            IDSetSwitch(&PWMScalerSP, nullptr);

            return true;

        }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool FCUSB::getStatus()
{
    // 2 bytes response
    uint8_t status[2] = {0};

    int rc = hid_read(handle, status, 2);

    if (rc < 0)
    {
        LOGF_ERROR("getStatus: Error reading from FCUSB to device (%s)", hid_error(handle));
        return false;
    }

    LOGF_DEBUG("RES <%#02X %#02X>", status[0], status[1]);

    // Motor Status
    uint8_t motor = status[0] & 0x3;
    // 0x2 and 0x3 are identical
    if (motor == 0x3) motor = 0x2;
    MotorBits newMotorStatus = static_cast<MotorBits>(motor);
    if (newMotorStatus != motorStatus)
    {
        motorStatus = newMotorStatus;
        switch (motorStatus)
        {
            case MOTOR_OFF:
                LOG_INFO("Motor is off.");
                break;

            case MOTOR_REV:
                LOG_INFO("Motor is moving backwards.");
                break;

            case MOTOR_FWD:
                LOG_INFO("Motor is moving forward.");
                break;

        }
    }

    uint8_t pwm   = (status[0] & 0xC0) >> 6 ;
    // 0x2 and 0x3 are identical
    if (pwm == 0x3) pwm = 0x2;
    PWMBits newPWMStatus = static_cast<PWMBits>(pwm);
    if (newPWMStatus != pwmStatus)
    {
        pwmStatus = newPWMStatus;
        switch (pwmStatus)
        {
            case PWM_1_1:
                LOG_INFO("PWM Scaler is 1:1");
                break;

            case PWM_1_4:
                LOG_INFO("PWM Scaler is 1:4");
                break;

            case PWM_1_16:
                LOG_INFO("PWM Scaler is 1:16");
                break;
        }

        IUResetSwitch(&PWMScalerSP);
        PWMScalerS[pwmStatus].s = ISS_ON;
        IDSetSwitch(&PWMScalerSP, nullptr);
    }

    // Update speed (PWM) if it was changed.
    if (fabs(FocusSpeedN[0].value - status[1]) > 0)
    {
        FocusSpeedN[0].value = status[1];
        LOGF_DEBUG("PWM: %d%", FocusSpeedN[0].value);
        IDSetNumber(&FocusSpeedNP, nullptr);
    }

    return true;
}

bool FCUSB::AbortFocuser()
{
    motorStatus = 0;
    return setStatus();
}

bool FCUSB::SetFocuserSpeed(int speed)
{
    targetSpeed = speed;
    return setStatus();
}

IPState FCUSB::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(dir);
    INDI_UNUSED(speed);
    INDI_UNUSED(duration);
    // TODO
    return IPS_ALERT;
}

bool FCUSB::setStatus()
{
    uint8_t command[2] = {0};

    command[0] |= motorStatus;
    // Forward (Green) - Reverse (Red)
    command[0] |= (motorStatus == MOTOR_REV) ? FC_LED_RED : 0;
    // PWM
    command[0] |= (pwmStatus << 6);

    command[1] = static_cast<uint8_t>(targetSpeed);

    int rc = hid_write(handle, command, 2);
    if (rc < 0)
    {
        LOGF_DEBUG("Setting state failed (%s)", hid_error(handle));
        return false;
    }

    return true;
}

bool FCUSB::saveConfigItems(FILE * fp)
{
    Focuser::saveConfigItems(fp);

    IUSaveConfigSwitch(fp, &PWMScalerSP);

    return true;
}
