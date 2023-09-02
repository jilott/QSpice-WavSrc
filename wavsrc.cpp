/*******************************************************************************
 * wavsrc.cpp -- QSpice C-Block component to read *.WAV file data as a voltage
 * source.
 *
 * Revision History:
 *  2023.08.26 - Proof of Concept (POC).
 *  2023.08.30 - Added support for 2-channel (stereo) files, passing filename
 *               from component attribute, and looping through the file more
 *               than once.  Also, added some debugging logging stuff.
 *  2023.09.01 - Added Vref input and gain parameters/attributes. Improved
 *               debugging/logging support.
 *
 * Limitations:
 *   Component supports only basic 16-bit PCM-encoded wave mono/stereo files.
 *
 * Known Issues:
 * * The debugging/logfile stuff will almost certainly break for multi-instance
 *   components. Possible solutions include a static shared file handle at the
 *   DLL level. We'd need the messages to include instance info (pointer
 *   address) to distinguish between messages from the multiple instances. I've
 *   not thought this out and, for now, assume that this is useful for only the
 *   first instance.
 * * I'm blissfully ignorant of any other issues as I write this.
 *
 * Potential Improvements:
 * * The debugging messages do not contain timestamps (i.e., system timestamps).
 *   Do we need this?  Seems overkill and timestamp stuff has a lot of overhead.
 *
 * Use this code at your own risk.  And please share any corrections or
 * improvements.
 ******************************************************************************/
// To build with Digital Mars C++ Compiler: dmc -mn -WD wavsrc.cpp kernel32.lib
// or open the component source in QSpice, right-click, compile...

#include <stdio.h>
#include <malloc.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <limits.h>
#include "wavsrc.h"

// basic version information
#define VERINFO "v0.1"
#define VERSTR  "WavSrc " VERINFO

// to generate a debugging log file, set the following #define to nonzero.  the
// logging process does not attempt to handle errors elegantly and may fail
// without clear indications.  definitely not intended for production use.
#define DBGLOG 1

/*
 * standard QSpice template-generated parameter data structure
 */
union uData {
  bool                   b;
  char                   c;
  unsigned char          uc;
  short                  s;
  unsigned short         us;
  int                    i;
  unsigned int           ui;
  float                  f;
  double                 d;
  long long int          i64;
  unsigned long long int ui64;
  char                  *str;
  unsigned char         *bytes;
};

// #undef pin names lest they collide with names in any header file(s) you might
// include. (could use namespaces if DMC.exe supports them)
#undef CH1
#undef CH2
#undef Vref

/*******************************************************************************
 * constants & defines for convenience
 ******************************************************************************/
#define FileClosed 0
#define FileOpen   1
#define FileError  -1

const char *MsgBadRead   = "Unexpected error reading WAV file (\"%s\").\n";
const char *MsgBadFormat = "Unsupported WAV format in file \"%s\"\n";
const char *MsgBadOpen =
    "Unexpected error opening WAV file (\"%s\").  File not found or cannot be "
    "opened.\n)";

/*******************************************************************************
 * Per-instance data.  The QSpice template generator gives this structure a
 * unique name based on the C-Block mocule name for reasons that excape me.
 ******************************************************************************/
struct InstData {
  FILE  *logFile;          // for debugging
  FILE  *file;             // file stream pointer for WAV data
  int    fileState;        // 0 = closed; -1 = error; 1 = open
  fpos_t startOfData;      // file position of start of data for looping
  int    sampleCnt;        // # of samples read so far
  int    nbrSamples;       // # of samples in file
  double lastCh1;          // last normalized value read/output of channel 1
  double lastCh2;          // last normalized value read/output of channel 2
  double nextSampleTime;   // time to fetch next sample
  double maxAmplitude;     // max input amplitude for normalization to +/-1.0
  double sampleTimeIncr;   // 1 / sample frequency
  int    nbrChannels;      // number of channels per sample
  int    maxLoops;         // number of times to loop through samples
  int    loopCnt;          // number of loops so far
  double gain;             // output gain to apply to normalized values
};

#if DBGLOG
#define LogFileName ".\\wavsrc.log"

void msg(InstData *, int, const char *, ...);   // need forward decl

void logMsg(InstData *inst, int lineNbr, const char *fmt, ...) {
  fprintf(inst->logFile, VERSTR " @%d: ", lineNbr);
  va_list args = {0};
  va_start(args, fmt);
  vfprintf(inst->logFile, fmt, args);
  va_end(args);
  fflush(inst->logFile);
}

void openLog(InstData *inst) {
  // open to overwrite
  if ((bool)(inst->logFile = fopen(LogFileName, "w"))) {
    logMsg(inst, __LINE__, "*** Start of Log ***\n");
    msg((InstData *)nullptr, __LINE__,
        "Is in debugging mode and logging to \"%s\"...\n", LogFileName);
  } else
    msg((InstData *)nullptr, __LINE__,
        "Is in debugging mode but unable to log to \"%s\".\n", LogFileName);
}

void closeLog(InstData *inst) {
  logMsg(inst, __LINE__, "*** End of log ***\n");
  fclose(inst->logFile);
}
#else
#define openLog(x)           /*generates no code*/
#define closeLog(x)          /*generates no code*/
#define logMsg(x, y, z, ...) /*generates no code*/
#endif

// output text to QSpice Output window (stdout).  note that messages sent here
// are duplicated to the log if enabled and instance is valid.
void msg(InstData *inst, int lineNbr, const char *fmt, ...) {
  msleep(30);   // can't find where this is defined in DMC includes...
  fflush(stdout);
  printf(VERSTR " @%d: ", lineNbr);
  va_list args = {0};
  va_start(args, fmt);
  vprintf(fmt, args);
  if (inst) vfprintf(inst->logFile, fmt, args);
  if (inst) logMsg(inst, lineNbr, fmt, args);
  va_end(args);
  fflush(stdout);
  msleep(30);

  // // forward to log if instance is valid
  // va_list args2 = {0};
  // va_start(args2, fmt);
  // if (inst) { logMsg(inst, lineNbr, fmt, args2); }
  // va_end(args2);
}

/*******************************************************************************
 * forward decls
 ******************************************************************************/
void   initInst(InstData &, uData *);
void   getSample(InstData &, double, const char *);
double getSample16(InstData &, const char *);

/*******************************************************************************
 * QSpice-defined entry points
 ******************************************************************************/
// int DllMain() must exist and return 1 for a process to load the .DLL
// See https://docs.microsoft.com/en-us/windows/win32/dlls/dllmain for more
// information.
int __stdcall DllMain(void *module, unsigned int reason, void *reserved) {
  return 1;
}

/*
 * note:  The uData parameter passes an array of component port and attribute
 * data.  the template generator should be used to generate proper indexes
 * into this data if the component is changed in the schematic.
 */
extern "C" __declspec(dllexport) void wavsrc(
    InstData **opaque, double t, uData *data) {

  double      Vref     = data[0].d;     // input
  const char *filename = data[1].str;   // input parameter
  int         loops    = data[2].i;     // input parameter
  double      gain     = data[3].d;     // input parameter
  double     &CH1      = data[4].d;     // output
  double     &CH2      = data[5].d;     // output

  InstData *inst = *opaque;

  // allocate per-instance data if not already allocated
  if (!inst) {
    // allocate & clear the memory block
    *opaque = inst = (InstData *)calloc(1, sizeof(InstData));
    if (!inst) {
      // can't get memory so terminate simulation with prejudice
      msg(inst, __LINE__,
          "Unable to allocate instance memory.  Aborting simulation...\n");
      exit(1);
    }

    // log stuff
    openLog(inst);

    // initialize the instance data (open/parse file through header stuff)
    initInst(*inst, data);
  }

  // if the current sample has "expired", get next sample
  if (t >= inst->nextSampleTime) getSample(*inst, t, filename);

  // set component's out port values to current sample values
  CH1 = (inst->lastCh1 * gain) + Vref;
  CH2 = (inst->lastCh2 * gain) + Vref;
}

/*
 * QSpice MaxExtStepSize() -- "implement a good choice of max timestep size
 * that depends on InstData".  Not sure what that means or, in fact, exactly
 * when this gets called so...
 */
extern "C" __declspec(dllexport) double MaxExtStepSize(InstData *inst) {
  double stepSize = 1e308;   // heat death of the universe?

  // if file is open, set to sample-time increment
  if (inst->fileState == FileOpen) stepSize = inst->sampleTimeIncr;

  return stepSize;
}

/*
 * QSpice Trun() -- "limit the timestep to a tolerance if the circuit causes a
 * change in InstData".  Not sure what this means or, in fact, exactly when
 * this gets called so...
 */
extern "C" __declspec(dllexport) void Trunc(
    InstData *inst, double t, union uData *data, double *timestep) {
  // note:  offsets must be duplicated exactly as in wavsrc() if used
  // double      Vref     = data[0].d;     // input
  // const char *filename = data[1].str;   // input parameter
  // int         loops    = data[2].i;     // input parameter
  // double      gain     = data[3].d;     // input parameter
  // double     &CH1      = data[4].d;     // output
  // double     &CH2      = data[5].d;     // output

  if (t <= inst->nextSampleTime) *timestep = inst->nextSampleTime - t;
}

/*
 * QSpice Destroy() -- clean up upon end of simulation
 */
extern "C" __declspec(dllexport) void Destroy(InstData *inst) {
  closeLog(inst);
  fclose(inst->file);
  free(inst);
}

/*******************************************************************************
 * WavSrc component functions
 ******************************************************************************/
/*------------------------------------------------------------------------------
 * initInst() - opens the WAV file, parses through fmt chunk, and stops at
 * beginning of sample data.  initializes instance data for first data read.
 *----------------------------------------------------------------------------*/
void initInst(InstData &inst, uData *data) {
  // note:  offsets must be duplicated exactly as in wavsrc() if used
  // double      Vref     = data[0].d;     // input
  const char *filename = data[1].str;   // input parameter
  int         loops    = data[2].i;     // input parameter
  double      gain     = data[3].d;     // input parameter
  // double     &CH1      = data[4].d;     // output
  // double     &CH2      = data[5].d;     // output

  // default instance file state to file error
  inst.fileState = FileError;

  msg(&inst, __LINE__, "Using WAV file=\"%s\", loops=%d, gain=%f\n", filename,
      loops, gain);

  // open the WAV file
  if (!(bool)(inst.file = fopen(filename, "rb"))) {
    msg(&inst, __LINE__, MsgBadOpen, filename);
    return;
  }

  // read file header info
  size_t             bytes;
  WavFileHeaderChunk fileHdr;

  bytes = fread(&fileHdr, 1, sizeof(fileHdr), inst.file);
  if (bytes != sizeof(fileHdr)) {
    fclose(inst.file);
    msg(&inst, __LINE__, MsgBadRead, filename);
    return;
  }

  logMsg(&inst, __LINE__,
      "groupID = \"%4.4s\", chunkSize = %d (total size = %d), riffType = "
      "\"%4.4s\"\n",
      fileHdr.groupID, fileHdr.chunkSize, fileHdr.chunkSize + 8,
      fileHdr.riffType);

  // check header for supported file type
  if (memcmp(fileHdr.groupID, "RIFF", 4) ||
      memcmp(fileHdr.riffType, "WAVE", 4)) {
    fclose(inst.file);
    msg(&inst, __LINE__, MsgBadFormat, filename);
    return;
  }

  // grab next chunk header -- should be a format chunk...
  WavChunkHeader chunkHdr;
  bytes = fread(&chunkHdr, 1, sizeof(chunkHdr), inst.file);
  if (bytes != sizeof(chunkHdr)) {
    // either EOF or unexpected error, don't really care which
    msg(&inst, __LINE__, MsgBadRead, filename);
    fclose(inst.file);
    return;
  }

  logMsg(&inst, __LINE__, "Chunk Type: \"%4.4s\", Size: %d\n", chunkHdr.format,
      chunkHdr.chunkSize);

  // verify that it is a format chunk
  if (memcmp(chunkHdr.format, "fmt ", 4)) {
    msg(&inst, __LINE__, MsgBadFormat, filename);
    fclose(inst.file);
    return;
  }

  // verify that the format chunk size is acceptable/expected
  switch (chunkHdr.chunkSize) {
  case 16:
  case 18:
  case 40:
    break;
  default:
    msg(&inst, __LINE__, MsgBadFormat, filename);
    fclose(inst.file);
    return;
  }

  // read/save format chunk data in instance data
  WavFmtChunk fmtChunk;
  bytes = fread(&fmtChunk, 1, chunkHdr.chunkSize, inst.file);
  if (bytes != chunkHdr.chunkSize) {
    // either EOF or unexpected error, don't really care which
    msg(&inst, __LINE__, MsgBadRead, filename);
    fclose(inst.file);
    return;
  }

  logMsg(&inst, __LINE__,
      "Format Chunk Data:\n"
      "  Format Code: %d\n"
      "  # of Channels: %d\n"
      "  Samples Per Second: %d\n"
      "  Avg Bytes Per Second: %d\n"
      "  Block Alignment:  %d\n"
      "  Bits Per Sample: %d\n",
      fmtChunk.fmtCode, fmtChunk.nbrChannels, fmtChunk.samplesPerSec,
      fmtChunk.avgBytesPerSec, fmtChunk.blkAlign, fmtChunk.bitsPerSample);

  // validate allowed format -- only 16-bit PCM is supported for now
  if (fmtChunk.fmtCode != FmtPCM) {
    msg(&inst, __LINE__, MsgBadFormat, filename);
    fclose(inst.file);
    return;
  }

  // only mono or stereo allowed
  if (fmtChunk.nbrChannels > 2) {
    msg(&inst, __LINE__, MsgBadFormat, filename);
    fclose(inst.file);
    return;
  }

  // set amplitude nomalization for bit depth -- commented code remains for
  // possible (unlikely) component enhancements
  switch (fmtChunk.bitsPerSample) {
    //	case 8: maxAmplitude = 0x7F; break;
  case 16:
    inst.maxAmplitude = 0x7fff;
    break;
    //	case 20: inst.maxAmplitude = 0x07FFFF; break;
    //	case 24: inst.maxAmplitude = 0x7FFFFF; break;
    //	case 32: inst.maxAmplitude = 0x7FFFFFFF; break;
  default:
    msg(&inst, __LINE__, MsgBadFormat, filename);
    fclose(inst.file);
    return;
  }

  // finally, get the data chunk (should be next)
  bytes = fread(&chunkHdr, 1, sizeof(chunkHdr), inst.file);
  if (bytes != sizeof(chunkHdr)) {
    // either EOF or unexpected error, don't really care which
    msg(&inst, __LINE__, MsgBadRead, filename);
    fclose(inst.file);
    return;
  }

  logMsg(&inst, __LINE__, "Chunk Type: \"%4.4s\", Size: %d\n", chunkHdr.format,
      chunkHdr.chunkSize);

  // if not "data", not expected/handled
  if (memcmp(chunkHdr.format, "data", 4)) {
    msg(&inst, __LINE__, MsgBadFormat, filename);
    fclose(inst.file);
    return;
  }

  // save values in instance data
  inst.nbrChannels    = fmtChunk.nbrChannels;
  inst.nbrSamples     = chunkHdr.chunkSize / 2 / inst.nbrChannels;
  inst.sampleTimeIncr = 1.0 / fmtChunk.samplesPerSec;
  inst.nextSampleTime = 0.0;
  inst.maxLoops = loops < 1 ? INT_MAX : loops;   // technically not infinity
  inst.lastCh1 = inst.lastCh2 = 0.0;
  inst.gain                   = gain;

  // logMsg(&inst, __LINE__, "SamplesPerSecond = %d, nbrSamples = %d\n",
  //     fmtChunk.samplesPerSec, inst.nbrSamples);

  // in theory, the file is positioned at the start of the data.  save the
  // position for looping...
  if (fgetpos(inst.file, &inst.startOfData)) {
    // error getting file position
    msg(&inst, __LINE__, MsgBadRead, filename);
    inst.fileState = FileError;
    fclose(inst.file);
    inst.lastCh1 = inst.lastCh2 = 0.0;
    return;
  }

  // in theory, we're ready to start reading samples
  inst.fileState = FileOpen;
}

/*------------------------------------------------------------------------------
 * getData() - gets the next sample(s) from the file.
 * *----------------------------------------------------------------------------*/
void getSample(InstData &inst, double t, const char *filename) {
  // default sample values
  inst.lastCh1 = inst.lastCh2 = 0.0;

  // have we reached the end of data?
  if (inst.sampleCnt >= inst.nbrSamples) {
    // a loop finished
    inst.loopCnt++;
    inst.sampleCnt = 0;

    // do we have more loops to do?
    if (inst.loopCnt >= inst.maxLoops) {
      // nope
      fclose(inst.file);
      inst.fileState      = FileClosed;
      inst.nextSampleTime = 1e308;   // we won't be reading again anytime soon
      return;
    }

    // reposition file to start of data
    if (fsetpos(inst.file, &inst.startOfData)) {
      // file positioning error
      fclose(inst.file);
      inst.fileState = FileClosed;
      return;
    }
  }

  // get first channel 16-bit sample
  inst.lastCh1 = inst.lastCh2 = getSample16(inst, filename);

  // if stereo, get the other channel sample
  if (inst.nbrChannels == 2) inst.lastCh2 = getSample16(inst, filename);

  // calculate next sample time adjusting for loop count
  inst.nextSampleTime = ((inst.loopCnt * inst.nbrSamples) + ++inst.sampleCnt) *
      inst.sampleTimeIncr;

  // temporary debugging
  // if (inst.sampleCnt < 5) {
  //   logMsg(&inst, __LINE__,
  //       "Sample %d, t=%14e, ch1=%f, ch2=%f, nextSampleTime=%14e\n",
  //       inst.sampleCnt, t, inst.lastCh1, inst.lastCh2, inst.nextSampleTime);
  // }
}

/*------------------------------------------------------------------------------
 * getSample16() - gets the next 16-bit value from the file and normalizes it
 * to +/-1.0.
 *----------------------------------------------------------------------------*/
double getSample16(InstData &inst, const char *filename) {
  int16_t sampleVal;

  int bytes = fread(&sampleVal, 1, sizeof(int16_t), inst.file);
  if (bytes < sizeof(sampleVal)) {
    // read failed
    inst.fileState = FileError;
    msg(&inst, __LINE__, MsgBadRead, filename);
    return 0.0;
  }

  return sampleVal / inst.maxAmplitude;
}
/*==============================================================================
 * EOF wavsrc.cpp
 *============================================================================*/
