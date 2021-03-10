#include "CAN_Qt.h"

extern CAN_HandleTypeDef hcan1;

extern CAN_param_t CAN_param;

void CAN_filterConfig(uint32_t* canFilter){
	CAN_FilterTypeDef filterConfig;
	CAN_FilterTypeDef filterConfig1;
	//skala 32 bit jest spoko do Ext ID, do Std ID najlepiej 16 bit, wtedy mamy 2x wiecej ID 
	//w skali 16 bit high i low oznacza 2 osobne ID
	//w skali 32 bit high i low oznacza MSB i LSB danego 32 bitowego rejestru
	//w trybie ID LIST oraz skali 16 bit w kazdym banku mozna ustawic 4 filtry Std ID
	//w trybie ID LIST FilterMask oraz FilterId dotyczy roznych ID
	//w trybie ID MASK FilterMask dotyczy maski a FilterId dotyczy ID
	filterConfig.FilterBank = 1;
	filterConfig.FilterActivation = CAN_FILTER_DISABLE;
	filterConfig.FilterFIFOAssignment = 0;
	filterConfig.FilterIdHigh = (canFilter[0] << 5);
	filterConfig.FilterIdLow = (canFilter[1] << 5);
	filterConfig.FilterMaskIdHigh = (canFilter[2] << 5);
	filterConfig.FilterMaskIdLow = (canFilter[3] << 5);
	filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
	
	filterConfig1.FilterBank = 0;
	filterConfig1.FilterActivation = CAN_FILTER_ENABLE;
	filterConfig1.FilterFIFOAssignment = 0;
	filterConfig1.FilterIdHigh = (0x000 << 5);
	filterConfig1.FilterIdLow = (0x001 << 5);
	filterConfig1.FilterMaskIdHigh = (0x001 << 5);
	filterConfig1.FilterMaskIdLow = (0x001 << 5);
	filterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig1.FilterScale = CAN_FILTERSCALE_16BIT;

	HAL_CAN_ConfigFilter(&hcan1, &filterConfig1);
}

void CANFilterActivate(uint32_t filterBank, bool active)
{
	uint32_t filternbrbitpos;
	
	filternbrbitpos = (uint32_t)1 << (filterBank & 0x1FU);
	
	if(true == active){
		SET_BIT(hcan1.Instance->FA1R, filternbrbitpos);
	}
	else 
		CLEAR_BIT(hcan1.Instance->FA1R, filternbrbitpos);
}

void CAN_Speed_Change(CAN_param_t* CAN_param)
{
	HAL_CAN_Stop(&hcan1); //Stop the CAN module and enable access to configuration registers.
	/* Set the bit timing register */
	uint32_t reg;
	CLEAR_BIT(hcan1.Instance->BTR, 0);
	CLEAR_BIT(hcan1.Instance->BTR, 1);
	CLEAR_BIT(hcan1.Instance->BTR, 2);
	CLEAR_BIT(hcan1.Instance->BTR, 4);
	CLEAR_BIT(hcan1.Instance->BTR, 8);
	CLEAR_BIT(hcan1.Instance->BTR, 16);
	reg = READ_REG(hcan1.Instance->BTR);
	WRITE_REG(hcan1.Instance->BTR, (uint32_t)( reg | (CAN_param->prescalerCAN - 1U)));
	HAL_CAN_Start(&hcan1);
	CAN_param->pasmoCAN = 0;	
}





