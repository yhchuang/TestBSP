;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2015 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


    AREA _image, DATA, READONLY

    EXPORT  LoaderImage1Base
    EXPORT  LoaderImage1Limit
    
    ALIGN   4
        
LoaderImage1Base
	IF      :DEF:__I91500__
	INCBIN ./KEIL/I91500/FMC_LD.bin
	ELSE
	INCBIN ./KEIL/I91500/FMC_LD.bin
	ENDIF
LoaderImage1Limit

    END