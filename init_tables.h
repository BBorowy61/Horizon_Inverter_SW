/********************************************************************/
/*	Parameter initialization tables									*/
/********************************************************************/
/********************************************************************/
/*		Revision history											*/
/********************************************************************/
/*
2008-08-19	add 400V voltage rating
2008-10-24	change burden values for 375kw and above
2008-12-17	add 1 MW rating
Solstice version
2009-10-15	delete 600 kW (37,38) and 1 MW (39) ratings
			change number of temp feedbacks for 500 kW from 8 to 6
*/


const far int ifbk_ratio_table[RATINGS_SIZE][KISTR+1]={
//	 IL   II  IN IDC  IGND ISTR  
	 200,2000,33,2000,667,2000,


	 300,2000,33,2000,667,2000,			// 80/265		// 80kw Solstice

	 300,2000,33,2000,667,2000,			// 100/(480v or 320v)		// 100kw Solstice

	 500,2000,33,2000,667,2000,			// 100/ (240v or 208v)		// 100kw Solstice
	
	 300,2000,33,2000,667,2000,			// 125/400		// 125kw Solstice

	 2000,5000,33,5000,667,2000,			//  (400/265)	// 500kw Solstice

	 2000,5000,33,5000,667,2000,			//  (500/ (480v or 320 v))	// 500kw Solstice

};		


const far int ifbk_burden_table[RATINGS_SIZE][KISTR+1]={
//	 IL II  IN IDC IGND ISTR  
//	50,200,100,150,600,500,
	
	39,300,100,300,600,330,

	20,200,100,100,600,330,			// 80/265 	// 80kw Solstice

	20,200,100,100,600,330,			// 100/480 	// 100kw Solstice

	20,200,100,100,600,330,			// 100/240v or 208v 	// 100kw Solstice

	20,200,100,100,600,330,			// 125/400	// 125kw Solstice

	39,100,100,62,600,330,			//  (400/265)	// 400kw Solstice

	39,100,100,62,600,330,			//  (500/480)	// 500kw Solstice

	};			



