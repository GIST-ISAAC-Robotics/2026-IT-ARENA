"""선택형 공식 출발 후보: 확인한 배열 밖 빨간 차체와 광원을 분리한다.

RGB에서 얻은 배열만 사용한다. 실제 가림·빨강 전체 이력 누락을 해결하지
않으며 기존 크기/배열/시간/녹색 확인 조건을 상속한다.
"""
import cv2

from arena_autonomy.race_start import OfficialStartSignal, _red_bank


def bank_window(bank):
    """빨강 네 열과 초록 예상 열을 포함하는 픽셀 창. 부품 좌표를 받지 않는다."""
    left = bank['cx']-1.5*bank['pitch']
    slope = (bank['gy']-bank['cy'])/(bank['gx']-bank['cx'])
    left_y = bank['cy']+slope*(left-bank['cx'])
    margin = 1.3*bank['diameter']
    return (left-margin, min(left_y, bank['gy'])-bank['vertical']-margin,
            bank['gx']+margin, max(left_y, bank['gy'])+bank['vertical']+margin)


def intersects(rect, window):
    x, y, width, height = rect
    left, top, right, bottom = window
    return x <= right and x+width >= left and y <= bottom and y+height >= top


class LocalOfficialStartSignal(OfficialStartSignal):
    def _red_observation(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid, invalid = [], []
        for contour in contours:
            if cv2.contourArea(contour) <= 10:
                continue
            rect = cv2.boundingRect(contour)
            x, y, width, height = rect
            if 5 <= width <= 100 and 4 <= height <= 100 and .55 < width/height < 1.8:
                valid.append(((x+width/2, y+height/2, float(width)), rect))
            else:
                invalid.append(rect)
        blobs = [v[0] for v in valid]
        # 새 배열은 전체 유효 광원으로 기존 검사에 통과해야 한다. 부분 배열이나
        # 여러 배열에서 임의의 네 열을 골라내지 않는다.
        current = _red_bank(blobs)
        bank = current or (self.bank if self.red_frames >= self.config.red_confirmations else None)
        if bank is None:
            return blobs, bool(invalid)
        window = bank_window(bank)
        # 중심점만 제외하면 창 안으로 뻗은 물체를 놓친다. 연결 성분 전체 경계를 쓴다.
        # 광원 형태의 추가 빨강은 원래 정책처럼 계속 모호성으로 취급한다.
        # 지역화로 제외하는 것은 창 밖의 미분류 차체 조각뿐이다.
        return blobs, any(intersects(rect, window) for rect in invalid)
