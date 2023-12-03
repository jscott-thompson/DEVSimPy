"""
	Author: J. Scott Thompson (j.scott.thompson.12@gmail.com)
	Date: 10/26/2023
	Description: Uses a compression-based surrogate to compute Kolmogorov complexity metrics
    Depends: liblzma, numpy
"""

from collections.abc import Callable
from inspect import getsource
from lzma import compress
from numpy import log2

def computeKolmogorovComplexity(f: bytes) -> float:
    """
    Computes a compression-based surrogate of Kolmogorov complexity for some binary data f
    """
    assert type(f) is bytes
    return log2(len(compress(f)))

def computeNormalizedCompressionDistance(f: bytes, g: bytes) -> float:
    """
    Computes the normalized compression distance for some binary data f and g
    """
    assert type(f) is bytes
    assert type(g) is bytes
    kc_f = computeKolmogorovComplexity(f)
    kc_g = computeKolmogorovComplexity(g)
    kc_fg = computeKolmogorovComplexity(f+g)
    ncd = (kc_fg - min(kc_f, kc_g))/max(kc_f, kc_g)
    return ncd

def getKolmogorovComplexityStr(s: str) -> float:
    assert isinstance(s, str)
    return computeKolmogorovComplexity(s.encode('utf-8'))

def getKolmogorovComplexityFunc(f: Callable) -> float:
    assert callable(f)
    return getKolmogorovComplexityStr(getsource(f))
