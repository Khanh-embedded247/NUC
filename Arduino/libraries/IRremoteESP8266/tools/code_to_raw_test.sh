#! /bin/bash
CODE_TO_RAW=./code_to_raw
if [[ ! -x ${CODE_TO_RAW} ]]; then
  echo "'raw_to_code' failed to compile and produce an executable."
  exit 1
fi

function unittest_success()
{
  COMMAND=$1
  EXPECTED="$2"
  echo -n "Testing: \"${COMMAND}\" ..."
  OUTPUT="$(${COMMAND})"
  STATUS=$?
  FAILURE=""
  if [[ ${STATUS} -ne 0 ]]; then
    FAILURE="Non-Zero Exit status: ${STATUS}. "
  fi
  if [[ "${OUTPUT}" != "${EXPECTED}" ]]; then
    FAILURE="${FAILURE} Unexpected Output: \"${OUTPUT}\" != \"${EXPECTED}\""
  fi
  if [[ -z ${FAILURE} ]]; then
    echo " ok!"
    return 0
  else
    echo
    echo "FAILED: ${FAILURE}"
    return 1
  fi
}

read -r -d '' OUT << EOM
Code type:    4 (SONY)
Code bits:    12

uint16_t rawData[78] = {2400, 600,  1200, 600,  1200, 600,  1200, 600,  1200, 600,  600, 600,  1200, 600,  600, 600,  1200, 600,  600, 600,  600, 600,  600, 600,  600, 24600,  2400, 600,  1200, 600,  1200, 600,  1200, 600,  1200, 600,  600, 600,  1200, 600,  600, 600,  1200, 600,  600, 600,  600, 600,  600, 600,  600, 24600,  2400, 600,  1200, 600,  1200, 600,  1200, 600,  1200, 600,  600, 600,  1200, 600,  600, 600,  1200, 600,  600, 600,  600, 600,  600, 600,  600, 24600 };  // SONY F50
uint32_t address = 0x1;
uint32_t command = 0x2F;
uint64_t data = 0xF50;
EOM

unittest_success "${CODE_TO_RAW} --protocol Sony --code 0xf50 --bits 12" "${OUT}"

read -r -d '' OUT << EOM
Code type:    7 (SAMSUNG)
Code bits:    32

uint16_t rawData[68] = {4480, 4480,  560, 1680,  560, 1680,  560, 1680,  560, 560,  560, 560,  560, 560,  560, 560,  560, 560,  560, 1680,  560, 1680,  560, 1680,  560, 560,  560, 560,  560, 560,  560, 560,  560, 560,  560, 1680,  560, 560,  560, 560,  560, 1680,  560, 1680,  560, 560,  560, 560,  560, 1680,  560, 560,  560, 1680,  560, 1680,  560, 560,  560, 560,  560, 1680,  560, 1680,  560, 560,  560, 47040 };  // SAMSUNG E0E09966
uint32_t address = 0x7;
uint32_t command = 0x99;
uint64_t data = 0xE0E09966;
EOM

unittest_success "${CODE_TO_RAW} --protocol SAMSUNG --code 0xE0E09966" "${OUT}"

read -r -d '' OUT << xEOMx
Code type:    18 (KELVINATOR)
Code bits:    128
Description:  Power: On, Mode: 1 (Cool), Temp: 27C, Fan: 1 (Low), Turbo: Off, Quiet: Off, XFan: On, Ion: Off, Light: Off, Swing(H): Off, Swing(V): 0 (Off)

uint16_t rawData[280] = {9010, 4504,  680, 1530,  680, 510,  680, 510,  680, 1530,  680, 1530,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 1530,  680, 510,  680, 1530,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 510,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 510,  680, 1530,  680, 510,  680, 510,  680, 1530,  680, 510,  680, 19974,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 1530,  680, 1530,  680, 39950,  9010, 4504,  680, 1530,  680, 510,  680, 510,  680, 1530,  680, 1530,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 1530,  680, 510,  680, 1530,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 510,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 1530,  680, 1530,  680, 510,  680, 510,  680, 1530,  680, 510,  680, 19974,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 510,  680, 1530,  680, 1530,  680, 1530,  680, 1530,  680, 39950 };  // KELVINATOR
uint8_t state[16] = {0x19, 0x0B, 0x80, 0x50, 0x00, 0x00, 0x00, 0xE0, 0x19, 0x0B, 0x80, 0x70, 0x00, 0x00, 0x10, 0xF0};
xEOMx

unittest_success "${CODE_TO_RAW} --protocol KELVINATOR --code 0x190B8050000000E0190B8070000010F0" "${OUT}"
