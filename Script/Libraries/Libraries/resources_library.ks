
//densities in 1000 kg per litre

DECLARE FUNCTION res_dens_init {
	parameter lexx.

	LOCAL res_density IS LEXICON(
		"Aerozine50",  0.0009,
		"AK20",    0.001499,
		"AK27",    0.001494,
		"Aniline",    0.00102,
		"AvGas",    0.000719,
		"CaveaB",    0.001501,
		"ClF3",    0.00177,
		"ClF5",    0.0019,
		"Diborane",    0.000421,
		"Ethane",    0.000544,
		"Ethanol",    0.000789,
		"Ethanol75",    0.00084175,
		"Ethanol90",    0.0008101,
		"Ethylene",    0.000568,
		"FLOX30",    0.0012517,
		"FLOX70",    0.0013993,
		"FLOX88",    0.0014657,
		"Furfuryl",    0.00113,
		"Helium",    0.0000001786,
		"HNIW",    0.002044,
		"HTP",    0.001431,
		"HTPB",    0.00177,
		"Hydrazine",    0.001004,
		"Hydyne",    0.00086,
		"IRFNA-III",    0.001658,
		"IRFNA-IV",    0.001995,
		"IWFNA",    0.001513,
		"Kerosene",    0.00082,
		"LeadBallast",    0.01134,
		"LqdFluorine",    0.001505,
		"LqdHydrogen",	0.00007085000,
		"LqdMethane",    0.00042561,
		"LqdOxygen",    0.001141,
		"Methane",    0.000000717,
		"Methanol",    0.0007918,
		"MMH",    0.00088,
		"MON1",    0.001429,
		"MON3",    0.001423,
		"MON10",    0.001407,
		"MON15",    0.001397,
		"MON20",    0.00138,
		"MON25",	0.00138,
		"NGNC",    0.0016,
		"N2F4",    0.001604,
		"Nitrogen",    0.000001251,
		"NitrousOxide",    0.00000196,
		"NTO",    0.00145,
		"OF2",    0.0019,
		"PBAN",    0.001772,
		"Pentaborane",    0.000618,
		"PSPC",    0.00174,
		"Syntin",    0.000851,
		"TEATEB",    0.00070031,
		"Tonka250",    0.000873,
		"Tonka500",    0.000811,
		"UDMH",    0.000791,
		"UH25",    0.000829
	).
	
	
	for reskey in lexx:KEYS {
		SET lexx[reskey] TO res_density[reskey].
	}
	
	return lexx.
	
}