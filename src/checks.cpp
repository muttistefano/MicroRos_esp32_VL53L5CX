#define RCCHECK(fn)                                                    \
  {                                                                    \
    rcl_ret_t temp_rc = fn;                                            \
    if ((temp_rc != RCL_RET_OK)) {                                     \
      printf("Failed status on line %d: %d. Message: %s, Aborting.\n", \
             __LINE__, (int)temp_rc, rcl_get_error_string().str);      \
      error_loop(temp_rc);                                             \
    }                                                                  \
  }

#define RCSOFTCHECK(fn)                                               \
  {                                                                   \
    rcl_ret_t temp_rc = fn;                                           \
    if ((temp_rc != RCL_RET_OK)) {                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, \
             (int)temp_rc);                                           \
    }                                                                 \
  }
