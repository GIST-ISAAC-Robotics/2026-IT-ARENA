import run_windows_tests as bootstrap
import sys
sys.path.insert(0,str(bootstrap.ROOT/'stage/tests'))
if __name__=='__main__':
    import pytest
    raise SystemExit(pytest.main(['-q','-p','no:cacheprovider','--basetemp='+str(bootstrap.ROOT/'revision_v2/testtmp'),
        str(bootstrap.ROOT/'stage/tests/test_vehicle_marker_duplicates.py'),
        str(bootstrap.ROOT/'stage/tests/test_vehicle_marker_regions.py'),
        '--junitxml='+str(bootstrap.ROOT/'revision_v2/windows_duplicates.xml')]))
