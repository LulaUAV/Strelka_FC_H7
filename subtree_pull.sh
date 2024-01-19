

git fetch git@github.com:Hover-Disaster/STM32_BMX055.git
git fetch git@github.com:Hover-Disaster/STM32_MS5611.git
git fetch git@github.com:Hover-Disaster/STM32_SD.git 
git fetch git@github.com:Hover-Disaster/STM32_LoRa.git 
git fetch git@github.com:Hover-Disaster/Strelka_State_Machine.git
git fetch git@github.com:s-park21/CHIRP.git

git subtree pull --prefix=libs/STM32_BMX055 git@github.com:Hover-Disaster/STM32_BMX055.git main --squash
git subtree pull --prefix=libs/STM32_MS5611 git@github.com:Hover-Disaster/STM32_MS5611.git main --squash
git subtree pull --prefix=libs/STM32_SD git@github.com:Hover-Disaster/STM32_SD.git main --squash
git subtree pull --prefix=libs/STM32_SD git@github.com:Hover-Disaster/STM32_LoRa.git main --squash
git subtree pull --prefix=libs/state_machine git@github.com:Hover-Disaster/Strelka_State_Machine.git main --squash
git subtree pull --prefix=libs/CHIRP git@github.com:s-park21/CHIRP.git main --squash