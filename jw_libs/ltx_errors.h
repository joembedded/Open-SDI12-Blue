/*******************************************************************************
* LTX_ERRORS.H
*
*******************************************************************************/


#define NRF_ERROR_OWN_BASE_NUM (0xF000) // Ab hier eigene Fehler (spaeter auslagern)
#define OWN_ERR_BUFFERSIZE NRF_ERROR_OWN_BASE_NUM + 1

#define LOC_ERROR_CHECK(res)            \
    {                                   \
        if (res != NRF_SUCCESS)         \
            local_error(res, __LINE__); \
    }

//

