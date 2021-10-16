#ifndef _SUBCOS_FSM_H
#define _SUBCOS_FSM_H

#define FSM_MAX_STATES  255

typedef enum fsm_err {
    FSM_OK,
} fsm_err_t;

typedef struct cosmic_state {
    char *name;
    int (*callback)(void);
    int transitions[FSM_MAX_STATES];
} cosmic_state_t;

typedef struct cosmic_fsm {
    char *name;
    cosmic_state_t states[FSM_MAX_STATES];
    int state_id[FSM_MAX_STATES];
} cosmic_fsm_t;


cosmic_fsm_t *fsm_create(const char *name);

fsm_err_t fsm_add_state(cosmic_fsm_t *fsm, cosmic_state_t *state, int state_id);
fsm_err_t fsm_del_state(cosmic_fsm_t *fsm, char *name);


#endif
