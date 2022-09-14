PROJECT( WinNTL )



IF( VERBOSE_FLAG  )
ADD_DEFINITIONS( -DVERBOSE )
ENDIF()


INCLUDE_DIRECTORIES(.
					"include"
					)
					
SET( WinNTL_SRC_FILES 
src/c_lip_impl.h
src/ctools.cpp
src/FacVec.cpp
src/FFT.cpp
src/fileio.cpp
src/g_lip_impl.h
src/G_LLL_FP.cpp
src/G_LLL_QP.cpp
src/G_LLL_RR.cpp
src/G_LLL_XD.cpp
src/GetTime.cpp
src/GF2.cpp
src/GF2E.cpp
src/GF2EX.cpp
src/GF2EXFactoring.cpp
src/GF2X.cpp
src/GF2X1.cpp
src/GF2XFactoring.cpp
src/GF2XVec.cpp
src/HNF.cpp
src/lip.cpp
src/LLL.cpp
src/LLL_FP.cpp
src/LLL_QP.cpp
src/LLL_RR.cpp
src/LLL_XD.cpp
src/lzz_p.cpp
src/lzz_pE.cpp
src/lzz_pEX.cpp
src/lzz_pEXFactoring.cpp
src/lzz_pX.cpp
src/lzz_pX1.cpp
src/lzz_pXCharPoly.cpp
src/lzz_pXFactoring.cpp
src/mat_GF2.cpp
src/mat_GF2E.cpp
src/mat_lzz_p.cpp
src/mat_lzz_pE.cpp
src/mat_poly_lzz_p.cpp
src/mat_poly_ZZ.cpp
src/mat_poly_ZZ_p.cpp
src/mat_RR.cpp
src/mat_ZZ.cpp
src/mat_ZZ_p.cpp
src/mat_ZZ_pE.cpp
src/pair_GF2EX_long.cpp
src/pair_GF2X_long.cpp
src/pair_lzz_pEX_long.cpp
src/pair_lzz_pX_long.cpp
src/pair_ZZ_pEX_long.cpp
src/pair_ZZ_pX_long.cpp
src/pair_ZZX_long.cpp
src/quad_float.cpp
src/RR.cpp
src/tools.cpp
src/vec_double.cpp
src/vec_GF2.cpp
src/vec_GF2E.cpp
src/vec_GF2XVec.cpp
src/vec_long.cpp
src/vec_lzz_p.cpp
src/vec_lzz_pE.cpp
src/vec_quad_float.cpp
src/vec_RR.cpp
src/vec_ulong.cpp
src/vec_vec_GF2.cpp
src/vec_vec_GF2E.cpp
src/vec_vec_long.cpp
src/vec_vec_lzz_p.cpp
src/vec_vec_lzz_pE.cpp
src/vec_vec_RR.cpp
src/vec_vec_ulong.cpp
src/vec_vec_ZZ.cpp
src/vec_vec_ZZ_p.cpp
src/vec_vec_ZZ_pE.cpp
src/vec_xdouble.cpp
src/vec_ZZ.cpp
src/vec_ZZ_p.cpp
src/vec_ZZ_pE.cpp
src/vec_ZZVec.cpp
src/WordVector.cpp
src/xdouble.cpp
src/ZZ.cpp
src/ZZ_p.cpp
src/ZZ_pE.cpp
src/ZZ_pEX.cpp
src/ZZ_pEXFactoring.cpp
src/ZZ_pX.cpp
src/ZZ_pX1.cpp
src/ZZ_pXCharPoly.cpp
src/ZZ_pXFactoring.cpp
src/ZZVec.cpp
src/ZZX.cpp
src/ZZX1.cpp
src/ZZXCharPoly.cpp
src/ZZXFactoring.cpp
	
	include/NTL/config.h
	)
	
	
ADD_LIBRARY( ${PROJECT_NAME} STATIC ${WinNTL_SRC_FILES} )
INSTALL( TARGETS ${PROJECT_NAME} DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

