
# STM32F767ZI_CubeIDE

> main file is saved at location '\<project-name>\Core\Src\main.c'

## USART - usart_virtualComPort
Tx Pin: PD8  
Rx Pin: PD9  
   _________________________   
  |           ______________|                       _______________
  |          |USART3        |                      | HyperTerminal |
  |          |              |                      |               |
  |          |           TX |______________________|RX             |
  |          |              |                      |               |
  |          |              |  Virtual Com Port    |               |             
  |          |              |                      |               |
  |          |           RX |______________________|TX             |          
  |          |              |                      |               |           
  |          |______________|                      |_______________|          
  |                         |                       
  |                         |                    
  |                         |                      
  |                         |                      
  |_STM32_Board_____________|                      
