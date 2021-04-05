#ifndef _INDEXER_H_
#define _INDEXER_H_

/*
* @struct: Indexer Data
*  @int mode: state of the indexer
*  @int maxVol: maximum voltage
*  @int minVol: minimum voltage
*/
extern struct INDEXER{
  int mode;
  int maxVol;
  int minVol;
  bool first;
} INDEXER_t;
extern INDEXER indexer;

/*
* @enum: Indexer Behaviors
*  @state INDX_IN: indexer rolls forward
*  @state INDX_OUT: indexer rolls backward
*  @state INDX_MOVE: indexer coasts
*/
enum indexerModes{
  INDX_IN,
  INDX_OUT,
  INDX_MOVE_IN,
  INDX_MOVE_OUT,
  INDX_BLUE,
  INDX_COAST,
  INDX_DO_NOTHING,
};

/*
* @task: State Machine for Indexer Motor
*/
extern void indexerTask(void* param);

#endif
