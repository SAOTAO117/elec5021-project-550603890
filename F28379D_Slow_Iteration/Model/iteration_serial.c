#include "iteration_serial.h"
#include "MW_f2837xD_includes.h"

#define MEASUREMENT_START ((uint16_T)'S')
#define MEASUREMENT_END ((uint16_T)'E')
#define RESULT_START ((uint16_T)'R')
#define RESULT_END ((uint16_T)'T')

#define LOAD_BEGIN ((uint16_T)'B')
#define LOAD_BEGIN_END ((uint16_T)'C')
#define LOAD_POINT ((uint16_T)'P')
#define LOAD_POINT_END ((uint16_T)'Q')
#define LOAD_COMMIT ((uint16_T)'G')
#define LOAD_COMMIT_END ((uint16_T)'H')
#define REPLAY_STOP ((uint16_T)'X')
#define REPLAY_STOP_END ((uint16_T)'Y')
#define STATUS_QUERY ((uint16_T)'J')
#define STATUS_QUERY_END ((uint16_T)'K')
#define STATUS_START ((uint16_T)'A')
#define STATUS_END ((uint16_T)'Z')

#define MEASUREMENT_BYTES (14U)
#define LOAD_BEGIN_BYTES (2U)
#define LOAD_POINT_BYTES (8U)
#define MAX_COMMAND_BYTES (14U)
#define PWM_REPLAY_MAX_SAMPLES (2048U)
#define Q13_HALF_DUTY ((int16_T)4096)

#define STATUS_OK (0U)
#define STATUS_BAD_LENGTH (1U)
#define STATUS_NOT_LOADING (2U)
#define STATUS_INDEX_ERROR (3U)
#define STATUS_INCOMPLETE (4U)
#define STATUS_LOAD_ERROR (5U)

static uint16_T parserState;
static uint16_T parserCommand;
static uint16_T parserEnd;
static uint16_T parserLength;
static uint16_T payloadIndex;
static uint16_T payload[MAX_COMMAND_BYTES];
static int16_T lastFrame[7];
static uint16_T sciInitialized;

static int16_T replayDuty[2][PWM_REPLAY_MAX_SAMPLES][3];
static volatile uint16_T activeBuffer;
static volatile uint16_T activeLength;
static volatile uint16_T replayIndex;
static volatile uint16_T replayActive;
static volatile uint16_T replayGeneration;
static volatile uint16_T loadBuffer;
static volatile uint16_T loadLength;
static volatile uint16_T loadIndex;
static volatile uint16_T loadInProgress;
static volatile uint16_T loadError;
static volatile uint16_T swapPending;

static void initializeSciA(void)
{
    EALLOW;
    DevCfgRegs.CPUSEL5.bit.SCI_A = 0U;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1U;
    asm(" NOP");

    SciaRegs.SCICCR.bit.STOPBITS = 0U;
    SciaRegs.SCICCR.bit.PARITY = 0U;
    SciaRegs.SCICCR.bit.PARITYENA = 0U;
    SciaRegs.SCICCR.bit.LOOPBKENA = 0U;
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0U;
    SciaRegs.SCICCR.bit.SCICHAR = 7U;
    SciaRegs.SCICTL1.bit.RXERRINTENA = 0U;
    SciaRegs.SCICTL1.bit.SWRESET = 1U;
    SciaRegs.SCICTL1.bit.TXENA = 1U;
    SciaRegs.SCICTL1.bit.RXENA = 1U;
    SciaRegs.SCIHBAUD.bit.BAUD = 0U;
    SciaRegs.SCILBAUD.bit.BAUD = 216U;
    SciaRegs.SCIPRI.bit.FREESOFT = 3U;
    SciaRegs.SCIFFCT.bit.ABDCLR = 0U;
    SciaRegs.SCIFFCT.bit.CDC = 0U;
    SciaRegs.SCIFFTX.bit.SCIRST = 1U;
    SciaRegs.SCIFFTX.bit.SCIFFENA = 1U;
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1U;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1U;
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1U;

    GpioCtrlRegs.GPBQSEL1.bit.GPIO43 = 3U;
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 3U;
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3U;
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0U;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 3U;
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3U;
    EDIS;
}

static void sendByte(uint16_T value)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST >= 16U) {
    }
    SciaRegs.SCITXBUF.bit.TXDT = value & 0x00FFU;
}

static uint16_T readWord(uint16_T offset)
{
    return payload[offset] | (payload[offset + 1U] << 8);
}

static void sendWord(uint16_T value)
{
    sendByte(value);
    sendByte(value >> 8);
}

static void sendStatus(uint16_T command, uint16_T status)
{
    sendByte(STATUS_START);
    sendByte(command);
    sendByte(status);
    sendByte(replayActive);
    sendWord(replayGeneration);
    sendWord(activeLength);
    sendWord(replayIndex);
    sendByte(STATUS_END);
}

static void beginLoad(void)
{
    uint16_T requestedLength = readWord(0U);

    loadInProgress = 0U;
    loadError = STATUS_OK;
    loadIndex = 0U;
    loadLength = 0U;

    if ((requestedLength < 2U) ||
            (requestedLength > PWM_REPLAY_MAX_SAMPLES)) {
        loadError = STATUS_BAD_LENGTH;
        sendStatus(LOAD_BEGIN, loadError);
        return;
    }

    loadBuffer = (activeBuffer == 0U) ? 1U : 0U;
    loadLength = requestedLength;
    loadInProgress = 1U;
    sendStatus(LOAD_BEGIN, STATUS_OK);
}

static void loadPoint(void)
{
    uint16_T pointIndex = readWord(0U);
    uint16_T phase;

    if (loadInProgress == 0U) {
        loadError = STATUS_NOT_LOADING;
        return;
    }
    if ((pointIndex != loadIndex) || (pointIndex >= loadLength)) {
        loadError = STATUS_INDEX_ERROR;
        return;
    }

    for (phase = 0U; phase < 3U; phase++) {
        replayDuty[loadBuffer][pointIndex][phase] =
            (int16_T)readWord(2U + 2U * phase);
    }
    loadIndex++;
}

static void commitLoad(void)
{
    uint16_T status = loadError;

    if (loadInProgress == 0U) {
        status = STATUS_NOT_LOADING;
    } else if (loadError != STATUS_OK) {
        status = STATUS_LOAD_ERROR;
    } else if (loadIndex != loadLength) {
        status = STATUS_INCOMPLETE;
    }

    if (status == STATUS_OK) {
        swapPending = 1U;
    }
    loadInProgress = 0U;
    sendStatus(LOAD_COMMIT, status);
}

static void stopReplay(void)
{
    replayActive = 0U;
    swapPending = 0U;
    loadInProgress = 0U;
    activeLength = 0U;
    replayIndex = 0U;
    sendStatus(REPLAY_STOP, STATUS_OK);
}

static void startParser(uint16_T command)
{
    parserCommand = command;
    payloadIndex = 0U;
    parserState = 1U;

    if (command == MEASUREMENT_START) {
        parserLength = MEASUREMENT_BYTES;
        parserEnd = MEASUREMENT_END;
    } else if (command == LOAD_BEGIN) {
        parserLength = LOAD_BEGIN_BYTES;
        parserEnd = LOAD_BEGIN_END;
    } else if (command == LOAD_POINT) {
        parserLength = LOAD_POINT_BYTES;
        parserEnd = LOAD_POINT_END;
    } else if (command == LOAD_COMMIT) {
        parserLength = 0U;
        parserEnd = LOAD_COMMIT_END;
    } else if (command == REPLAY_STOP) {
        parserLength = 0U;
        parserEnd = REPLAY_STOP_END;
    } else {
        parserLength = 0U;
        parserEnd = STATUS_QUERY_END;
    }

    if (parserLength == 0U) {
        parserState = 2U;
    }
}

static uint16_T isCommandStart(uint16_T value)
{
    return (value == MEASUREMENT_START) || (value == LOAD_BEGIN) ||
        (value == LOAD_POINT) || (value == LOAD_COMMIT) ||
        (value == REPLAY_STOP) || (value == STATUS_QUERY);
}

static void processCommand(int16_T frame[7], uint16_T *valid)
{
    uint16_T index;

    if (parserCommand == MEASUREMENT_START) {
        for (index = 0U; index < 7U; index++) {
            lastFrame[index] = (int16_T)readWord(2U * index);
            frame[index] = lastFrame[index];
        }
        *valid = 1U;
    } else if (parserCommand == LOAD_BEGIN) {
        beginLoad();
    } else if (parserCommand == LOAD_POINT) {
        loadPoint();
    } else if (parserCommand == LOAD_COMMIT) {
        commitLoad();
    } else if (parserCommand == REPLAY_STOP) {
        stopReplay();
    } else if (parserCommand == STATUS_QUERY) {
        sendStatus(STATUS_QUERY, STATUS_OK);
    }
}

void iteration_serial_frame_step(int16_T frame[7], uint16_T *valid)
{
    uint16_T byteValue;
    uint16_T index;

    *valid = 0U;

    if (sciInitialized == 0U) {
        initializeSciA();
        sciInitialized = 1U;
    }

    if (SciaRegs.SCIFFRX.bit.RXFFOVF == 1U) {
        SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1U;
        parserState = 0U;
        payloadIndex = 0U;
    }

    while (SciaRegs.SCIFFRX.bit.RXFFST > 0U) {
        byteValue = SciaRegs.SCIRXBUF.all & 0x00FFU;

        if (parserState == 0U) {
            if (isCommandStart(byteValue) != 0U) {
                startParser(byteValue);
            }
        } else if (parserState == 1U) {
            payload[payloadIndex] = byteValue;
            payloadIndex++;
            if (payloadIndex >= parserLength) {
                parserState = 2U;
            }
        } else {
            if (byteValue == parserEnd) {
                processCommand(frame, valid);
                parserState = 0U;
                payloadIndex = 0U;
            } else if (isCommandStart(byteValue) != 0U) {
                startParser(byteValue);
            } else {
                parserState = 0U;
                payloadIndex = 0U;
            }
        }
    }

    for (index = 0U; index < 7U; index++) {
        frame[index] = lastFrame[index];
    }
}

void iteration_serial_result_step(const int16_T result[6])
{
    uint16_T index;

    sendByte(RESULT_START);
    sendWord((uint16_T)lastFrame[0]);
    for (index = 0U; index < 6U; index++) {
        sendWord((uint16_T)result[index]);
    }
    sendByte(RESULT_END);
}

void pwm_replay_step(int16_T duty[3], uint16_T *active, uint16_T *index)
{
    uint16_T phase;

    if ((swapPending != 0U) &&
            ((replayActive == 0U) || (replayIndex == 0U))) {
        activeBuffer = loadBuffer;
        activeLength = loadLength;
        replayIndex = 0U;
        replayActive = 1U;
        swapPending = 0U;
        replayGeneration++;
    }

    if ((replayActive != 0U) && (activeLength > 0U)) {
        for (phase = 0U; phase < 3U; phase++) {
            duty[phase] = replayDuty[activeBuffer][replayIndex][phase];
        }
        *active = 1U;
        *index = replayIndex;
        replayIndex++;
        if (replayIndex >= activeLength) {
            replayIndex = 0U;
        }
    } else {
        duty[0] = Q13_HALF_DUTY;
        duty[1] = Q13_HALF_DUTY;
        duty[2] = Q13_HALF_DUTY;
        *active = 0U;
        *index = 0U;
    }
}
