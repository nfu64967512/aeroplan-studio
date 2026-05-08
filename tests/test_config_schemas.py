"""config.schemas 單元測試"""
import pytest

from config.schemas import (
    FlightParameters, StrikeParameters, SITLParameters,
)


class TestFlightParameters:
    def test_defaults_valid(self):
        FlightParameters().validate()   # 預設值應通過

    def test_invalid_altitude(self):
        with pytest.raises(ValueError, match='altitude'):
            FlightParameters(altitude=-10).validate()
        with pytest.raises(ValueError, match='altitude'):
            FlightParameters(altitude=10000).validate()

    def test_invalid_speed(self):
        with pytest.raises(ValueError, match='speed'):
            FlightParameters(speed=0).validate()

    def test_invalid_overlap(self):
        with pytest.raises(ValueError, match='overlap_rate'):
            FlightParameters(overlap_rate=0.8).validate()

    def test_invalid_vehicle(self):
        with pytest.raises(ValueError, match='vehicle_type'):
            FlightParameters(vehicle_type='helicopter').validate()

    def test_from_dict_ignores_unknown(self):
        fp = FlightParameters.from_dict({
            'altitude': 80, 'unknown_key': 'xyz', 'extra': 123,
        })
        assert fp.altitude == 80

    def test_roundtrip(self):
        fp1 = FlightParameters(altitude=75, vehicle_type='fixed_wing')
        fp2 = FlightParameters.from_dict(fp1.to_dict())
        assert fp1.altitude == fp2.altitude
        assert fp1.vehicle_type == fp2.vehicle_type


class TestStrikeParameters:
    def test_defaults_valid(self):
        StrikeParameters().validate()

    def test_dtot_requires_interval(self):
        with pytest.raises(ValueError, match='interval_sec'):
            StrikeParameters(timing_mode='DTOT', interval_sec=0).validate()

    def test_dtot_with_interval_ok(self):
        StrikeParameters(timing_mode='DTOT', interval_sec=15.0).validate()

    def test_stall_must_be_below_max(self):
        with pytest.raises(ValueError, match='max_speed'):
            StrikeParameters(stall_speed=80, max_speed=70).validate()

    def test_invalid_dive_angle(self):
        with pytest.raises(ValueError, match='max_dive_angle'):
            StrikeParameters(max_dive_angle=0).validate()
        with pytest.raises(ValueError, match='max_dive_angle'):
            StrikeParameters(max_dive_angle=95).validate()

    def test_update_revalidates(self):
        sp = StrikeParameters()
        with pytest.raises(ValueError):
            sp.update(timing_mode='DTOT')  # 沒給 interval → 重驗失敗


class TestSITLParameters:
    def test_defaults_valid(self):
        SITLParameters().validate()

    def test_invalid_vehicle(self):
        with pytest.raises(ValueError, match='vehicle'):
            SITLParameters(vehicle='submarine').validate()

    def test_instance_count_range(self):
        with pytest.raises(ValueError):
            SITLParameters(instance_count=0).validate()
        with pytest.raises(ValueError):
            SITLParameters(instance_count=100).validate()
