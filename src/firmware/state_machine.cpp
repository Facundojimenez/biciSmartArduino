#include<Arduino.h>
#include"definitions.h"

void state_machine()
{
  get_event();

  switch (currentState)
  {
  case STATE_WAITING_FOR_TRAINING:
    switch (currentEvent)
    {
    case EVENT_TRAINING_RECEIVED:
      showTrainingState("Received");
      currentState = STATE_READY_FOR_TRAINING;
      break;
    case EVENT_CONTINUE:
      showTrainingState("Not Received");
      currentState = STATE_WAITING_FOR_TRAINING;
      break;
    case EVENT_TRAINING_BUTTON:
      defaultTraining();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    default:
      break;
    }
    break;
  case STATE_READY_FOR_TRAINING:
    switch (currentEvent)
    {
    case EVENT_TRAINING_BUTTON:
      startTraining();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_CONTINUE:
      showTrainingState("Waiting to Start");
      sendTrainningState("WAITTING");
      currentState = STATE_READY_FOR_TRAINING;
      break;
    default:
      break;
    }
    break;
  case STATE_TRAINING_IN_PROGRESS:
    switch (currentEvent)
    {
    case EVENT_TRAINING_CONCLUDED:
      trainingFinished("Concluded");
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_TRAINING_BUTTON:
      showTrainingState("Paused");
      sendTrainningState("PAUSED");
      currentState = STATE_PAUSED_TRAINING;
      break;
    case EVENT_TRAINING_CANCELLED:
      trainingFinished("Cancelled");
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_PLAY_STOP_MEDIA_BUTTON:
      sendMusicComand("PLAY/STOP");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_NEXT_MEDIA_BUTTON:
      sendMusicComand("NEXT");
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_CONTINUE:
      updateTrainingState();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_VOLUME_CHANGE:
      updateVolume();
      break;
    default:
      break;
    }
    break;
  case STATE_PAUSED_TRAINING:
    switch (currentEvent)
    {
    case EVENT_TRAINING_BUTTON:
      resumeTraining();
      currentState = STATE_TRAINING_IN_PROGRESS;
      break;
    case EVENT_TRAINING_CANCELLED:
      trainingFinished("Cancelled");
      currentState = STATE_TRAINING_FINISHED;
      break;
    case EVENT_CONTINUE:
      offLed();
      currentState = STATE_PAUSED_TRAINING;
      break;
    default:
      break;
    }
    break;
  case STATE_TRAINING_FINISHED:
    switch (currentEvent)
    {
    case EVENT_TRAINING_RESTARTED:
      resetTraining();
      currentState = STATE_WAITING_FOR_TRAINING;
      break;
    case EVENT_CONTINUE:
      currentState = STATE_TRAINING_FINISHED;
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}
