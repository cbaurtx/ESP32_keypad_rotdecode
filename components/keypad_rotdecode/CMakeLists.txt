idf_component_register(SRCS "keypad_rotdecode.c"
                    INCLUDE_DIRS "include"
                    REQUIRES ulp)

set(ulp_app_name "ulp_debounce_decode")
set(ulp_s_sources "ulp/debounce_decode.S")
set(ulp_exp_dep_srcs "keypad_rotdecode.c")
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
