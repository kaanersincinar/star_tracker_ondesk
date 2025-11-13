# gimbal_move_validator.py
#
# Manyetik okuyucudan gelen veriler ile gimbal hareketini doğrulamak için
# bağımsız algoritma modülü.
#
# Kullanım mantığı:
# 1) Hareket öncesi encoder pozisyonlarını kaydet  (start_positions)
# 2) Gimbali komutla (X/Y/AZ vs. kaç mm döndürdün onu biliyorsun)
# 3) Hareket sonrası encoder pozisyonlarını kaydet (end_positions)
# 4) Bu modüle expected vs measured delta'ları ver → pass/fail + hata değerleri gelsin.

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class AxisMoveResult:
    axis: str
    commanded_delta_mm: float
    measured_delta_mm: float
    abs_error_mm: float
    rel_error_percent: Optional[float]
    passed: bool
    reason: str


@dataclass
class ValidationSummary:
    axis_results: Dict[str, AxisMoveResult]
    all_passed: bool


@dataclass
class ValidationConfig:
    # Küçük hareketlerde relatif hata saçmalamasın diye alt limit
    min_motion_for_percent_mm: float = 1.0

    # Mutlak hata limiti (mm)
    max_abs_error_mm: float = 0.1  # örnek: 0.1 mm

    # Yüzdesel hata limiti (%)
    max_rel_error_percent: float = 2.0  # örnek: %2


class GimbalMoveValidator:
    def __init__(self, config: ValidationConfig = ValidationConfig()):
        self.cfg = config

    def validate_axis(
        self,
        axis_name: str,
        commanded_delta_mm: float,
        measured_delta_mm: float,
    ) -> AxisMoveResult:
        abs_error = measured_delta_mm - commanded_delta_mm
        abs_error_mm = abs(abs_error)

        # Relatif hata: çok küçük hareketlerde anlamlı değil
        if abs(commanded_delta_mm) >= self.cfg.min_motion_for_percent_mm:
            rel_error_percent = (abs_error_mm / abs(commanded_delta_mm)) * 100.0
        else:
            rel_error_percent = None

        # PASS / FAIL kararı
        passed = True
        reasons = []

        if abs_error_mm > self.cfg.max_abs_error_mm:
            passed = False
            reasons.append(
                f"abs error {abs_error_mm:.4f} mm > limit {self.cfg.max_abs_error_mm:.4f} mm"
            )

        if rel_error_percent is not None and rel_error_percent > self.cfg.max_rel_error_percent:
            passed = False
            reasons.append(
                f"rel error {rel_error_percent:.2f}% > limit {self.cfg.max_rel_error_percent:.2f}%"
            )

        if not reasons:
            reasons.append("within limits")

        return AxisMoveResult(
            axis=axis_name,
            commanded_delta_mm=commanded_delta_mm,
            measured_delta_mm=measured_delta_mm,
            abs_error_mm=abs_error_mm,
            rel_error_percent=rel_error_percent,
            passed=passed,
            reason=" & ".join(reasons),
        )

    def validate_move(
        self,
        commanded_deltas_mm: Dict[str, float],
        measured_deltas_mm: Dict[str, float],
    ) -> ValidationSummary:
        """
        commanded_deltas_mm: {"X": 50.0, "Y": -10.0, "AZ": 0.0} gibi
        measured_deltas_mm:  manyetik okuyucudan hesapladığın delta'lar
        """
        axis_results: Dict[str, AxisMoveResult] = {}

        for axis, cmd_delta in commanded_deltas_mm.items():
            meas_delta = measured_deltas_mm.get(axis)
            if meas_delta is None:
                # O eksen için veri yoksa direkt FAIL yaz
                result = AxisMoveResult(
                    axis=axis,
                    commanded_delta_mm=cmd_delta,
                    measured_delta_mm=0.0,
                    abs_error_mm=abs(cmd_delta),
                    rel_error_percent=None,
                    passed=False,
                    reason="no measured data for axis",
                )
            else:
                result = self.validate_axis(axis, cmd_delta, meas_delta)

            axis_results[axis] = result

        all_passed = all(r.passed for r in axis_results.values())

        return ValidationSummary(
            axis_results=axis_results,
            all_passed=all_passed,
        )


def pretty_print_summary(summary: ValidationSummary) -> None:
    print("==== GIMBAL MOVE VALIDATION ====")
    for axis, res in summary.axis_results.items():
        rel_str = (
            f"{res.rel_error_percent:.2f}%"
            if res.rel_error_percent is not None
            else "N/A"
        )
        status = "PASS" if res.passed else "FAIL"

        print(
            f"[{axis}] {status} | "
            f"cmd: {res.commanded_delta_mm:+.4f} mm | "
            f"meas: {res.measured_delta_mm:+.4f} mm | "
            f"abs_err: {res.abs_error_mm:.4f} mm | "
            f"rel_err: {rel_str} | "
            f"{res.reason}"
        )

    print(f"OVERALL: {'PASS' if summary.all_passed else 'FAIL'}")


if __name__ == "__main__":
    # Küçük bir demo / self-test
    cfg = ValidationConfig(
        min_motion_for_percent_mm=1.0,
        max_abs_error_mm=0.1,
        max_rel_error_percent=2.0,
    )
    validator = GimbalMoveValidator(cfg)

    # Örnek: X eksenine 50 mm git dedin, Y eksenine -10 mm git dedin
    commanded = {
        "X": 50.0,
        "Y": -10.0,
        "AZ": 0.0,
    }

    # Manyetik okuyucudan ölçülen delta'lar (start/end farkı)
    measured = {
        "X": 49.92,
        "Y": -10.18,
        "AZ": 0.05,
    }

    summary = validator.validate_move(commanded, measured)
    pretty_print_summary(summary)
