ADD_LIBRARY(Math2
		../Common/Config.h
		MathDefs.h
		MatBase.h
		Mat.h
		Mat.cpp
		VecBase.h
		Vec.h
		Vec.cpp
		CDMath.cpp
		CDMath.h		
		SimMath.cpp
		SimMath.h
		Quaternion.h
		Quaternion.cpp
		VectorND.h
		VectorND.cpp
		MatrixNxM.h
		MatrixNxM.cpp
		
		CMakeLists.txt
)

SET_TARGET_PROPERTIES(Math2 PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE ${IBDS_STATIC_LIBRARY_FLAGS_RELEASE})   