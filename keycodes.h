#ifndef __KEYCODES__
#define __KEYCODES__

#define MMKEY_VOL_UP			0xE9
#define MMKEY_VOL_DOWN			0xEA
#define MMKEY_PLAYPAUSE			0xCD

PROGMEM const uint8_t KEYCODES_LAYOUT_US[96] = {
	0x2C,	/*	Space	*/
	0x1E,	/*	!		*/
	0x34,	/*	"		*/
	0x20,	/*	#		*/
	0x21,	/*	$		*/
	0x22,	/*	%		*/
	0x24,	/*	&		*/
	0x34,	/*	'		*/
	0x26,	/*	(		*/
	0x27,	/*	)		*/
	0x25,	/*	*		*/
	0x2E,	/*	+		*/
	0x36,	/*	,		*/
	0x2D,	/*	-		*/
	0x37,	/*	.		*/
	0x38,	/*	/		*/
	0x27,	/*	0		*/
	0x1E,	/*	1		*/
	0x1F,	/*	2		*/
	0x20,	/*	3		*/
	0x21,	/*	4		*/
	0x22,	/*	5		*/
	0x23,	/*	6		*/
	0x24,	/*	7		*/
	0x25,	/*	8		*/
	0x26,	/*	9		*/
	0x33,	/*	:		*/
	0x33,	/*	;		*/
	0x36,	/*	<		*/
	0x2E,	/*	=		*/
	0x37,	/*	>		*/
	0x38,	/*	?		*/
	0x1F,	/*	@		*/
	0x04,	/*	A		*/
	0x05,	/*	B		*/
	0x06,	/*	C		*/
	0x07,	/*	D		*/
	0x08,	/*	E		*/
	0x09,	/*	F		*/
	0x0A,	/*	G		*/
	0x0B,	/*	H		*/
	0x0C,	/*	I		*/
	0x0D,	/*	J		*/
	0x0E,	/*	K		*/
	0x0F,	/*	L		*/
	0x10,	/*	M		*/
	0x11,	/*	N		*/
	0x12,	/*	O		*/
	0x13,	/*	P		*/
	0x14,	/*	Q		*/
	0x15,	/*	R		*/
	0x16,	/*	S		*/
	0x17,	/*	T		*/
	0x18,	/*	U		*/
	0x19,	/*	V		*/
	0x1A,	/*	W		*/
	0x1B,	/*	X		*/
	0x1C,	/*	Y		*/
	0x1D,	/*	Z		*/
	0x2F,	/*	[		*/
	0x31,	/*	\		*/
	0x30,	/*	]		*/
	0x23,	/*	^		*/
	0x2D,	/*	_		*/
	0x35,	/*	`		*/
	0x04,	/*	a		*/
	0x05,	/*	b		*/
	0x06,	/*	c		*/
	0x07,	/*	d		*/
	0x08,	/*	e		*/
	0x09,	/*	f		*/
	0x0A,	/*	g		*/
	0x0B,	/*	h		*/
	0x0C,	/*	i		*/
	0x0D,	/*	j		*/
	0x0E,	/*	k		*/
	0x0F,	/*	l		*/
	0x10,	/*	m		*/
	0x11,	/*	n		*/
	0x12,	/*	o		*/
	0x13,	/*	p		*/
	0x14,	/*	q		*/
	0x15,	/*	r		*/
	0x16,	/*	s		*/
	0x17,	/*	t		*/
	0x18,	/*	u		*/
	0x19,	/*	v		*/
	0x1A,	/*	w		*/
	0x1B,	/*	x		*/
	0x1C,	/*	y		*/
	0x1D,	/*	z		*/
	0x2F,	/*	}		*/
	0x31,	/*	|		*/
	0x30,	/*	}		*/
	0x35 	/*	~		*/
};


PROGMEM const uint8_t KEYCODES_LAYOUT_DE[96] = {
	0x2C,	/* 	Space	*/
	0x1E,	/*	!		*/
	0x1F,	/*	"		*/
	0x31,	/*	#		*/
	0x21,	/*	$		*/
	0x22,	/*	%		*/
	0x23,	/*	&		*/
	0x31,	/*	'		*/
	0x25,	/*	(		*/
	0x26,	/*	)		*/
	0x30,	/*	*		*/
	0x30,	/*	+		*/
	0x36,	/*	,		*/
	0x38,	/*	-		*/
	0x37,	/*	.		*/
	0x24,	/*	/		*/
	0x27,	/*	0		*/
	0x1E,	/*	1		*/
	0x1F,	/*	2		*/
	0x20,	/*	3		*/
	0x21,	/*	4		*/
	0x22,	/*	5		*/
	0x23,	/*	6		*/
	0x24,	/*	7		*/
	0x25,	/*	8		*/
	0x26,	/*	9		*/
	0x37,	/*	:		*/
	0x2E,	/*	;		*/
	0x31,	/*	<		*/
	0x27,	/*	=		*/
	0x31,	/*	>		*/
	0x2E,	/*	?		*/
	0x14,	/*	@		*/
	0x04,	/*	A		*/
	0x05,	/*	B		*/
	0x06,	/*	C		*/
	0x07,	/*	D		*/
	0x08,	/*	E		*/
	0x09,	/*	F		*/
	0x0A,	/*	G		*/
	0x0B,	/*	H		*/
	0x0C,	/*	I		*/
	0x0D,	/*	J		*/
	0x0E,	/*	K		*/
	0x0F,	/*	L		*/
	0x10,	/*	M		*/
	0x11,	/*	N		*/
	0x12,	/*	O		*/
	0x13,	/*	P		*/
	0x14,	/*	Q		*/
	0x15,	/*	R		*/
	0x16,	/*	S		*/
	0x17,	/*	T		*/
	0x18,	/*	U		*/
	0x19,	/*	V		*/
	0x1A,	/*	W		*/
	0x1B,	/*	X		*/
	0x1D,	/*	Y		*/
	0x1c,	/*	Z		*/
	0x25,	/*	[		*/
	0x2E,	/*	\		*/
	0x26,	/*	]		*/
	0x35,	/*	^		*/
	0x38,	/*	_		*/
	0x2E,	/*	`		*/
	0x04,	/*	a		*/
	0x05,	/*	b		*/
	0x06,	/*	c		*/
	0x07,	/*	d		*/
	0x08,	/*	e		*/
	0x09,	/*	f		*/
	0x0A,	/*	g		*/
	0x0B,	/*	h		*/
	0x0C,	/*	i		*/
	0x0D,	/*	j		*/
	0x0E,	/*	k		*/
	0x0F,	/*	l		*/
	0x10,	/*	m		*/
	0x11,	/*	n		*/
	0x12,	/*	o		*/
	0x13,	/*	p		*/
	0x14,	/*	q		*/
	0x15,	/*	r		*/
	0x16,	/*	s		*/
	0x17,	/*	t		*/
	0x18,	/*	u		*/
	0x19,	/*	v		*/
	0x1A,	/*	w		*/
	0x1B,	/*	x		*/
	0x1D,	/*	y		*/
	0x1C,	/*	z		*/
	0x24,	/*	}		*/
	0x31,	/*	|		*/
	0x27,	/*	}		*/
	0x30	/*	~		*/
};


PROGMEM const uint8_t MODIFIER_SHIFT_LAYOUT_US[12] = {
	B01111111,	/*		Space	!	"	#	$	%	&	'		*/
	B11110000,	/*			(	)	*	+	,	-	.	/		*/
	B00000000,	/*			0	1	2	3	4	5	6	7		*/
	B00101010,	/*			8	9	:	;	<	=	>	?		*/
	B11111111,	/*			@	A	B	C	D	E	F	G		*/
	B11111111,	/*			H	I	J	K	L	M	N	O		*/
	B11111111,	/*			P	Q	R	S	T	U	V	W		*/
	B11100011,	/*			X	Y	Z	[	\	]	^	_		*/
	B00000000,	/*			`	a	b	c	d	e	f	g		*/
	B00000000,	/*			h	i	j	k	l	m	n	o		*/
	B00000000,	/*			p	q	r	s	t	u	v	w		*/
	B00011110,	/*			x	y	z	{	|	}	~	Lock	*/
};


PROGMEM const uint8_t MODIFIER_SHIFT_LAYOUT_DE[12] = {
	B01111111,	/*		Space	!	"	#	$	%	&	'		*/
	B11100001,	/*			(	)	*	+	,	-	.	/		*/
	B00000000,	/*			0	1	2	3	4	5	6	7		*/
	B00110111,	/*			8	9	:	;	<	=	>	?		*/
	B01111111,	/*			@	A	B	C	D	E	F	G		*/
	B11111111,	/*			H	I	J	K	L	M	N	O		*/
	B11111111,	/*			P	Q	R	S	T	U	V	W		*/
	B11100001,	/*			X	Y	Z	[	\	]	^	_		*/
	B10000000,	/*			`	a	b	c	d	e	f	g		*/
	B00000000,	/*			h	i	j	k	l	m	n	o		*/
	B00000000,	/*			p	q	r	s	t	u	v	w		*/
	B00000000,	/*			x	y	z	{	|	}	~	Lock	*/
};
	/*	The last bit, described as "Lock" above, tells about the function of the Caps	*
	 *	Lock key for your OS and layout. 0 means Caps Lock (only letters toggled), 1	*
	 *	means Shift Lock (all keys toggled). 											*/


#endif
