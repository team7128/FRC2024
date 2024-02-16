#include "subsystems/Subsystems.h"

Subsystems& Subsystems::GetInstance()
{
	static Subsystems instance;
	return instance;
}