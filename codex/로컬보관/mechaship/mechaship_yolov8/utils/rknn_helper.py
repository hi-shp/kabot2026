import cv2
import numpy as np


class RKNNHelper:
    def __init__(self, threshold, iou_threshold, img_size) -> None:
        self.threshold = threshold
        self.iou_threshold = iou_threshold
        self.img_size = img_size

        self.scale = 1.0
        self.pad_top = 0
        self.pad_left = 0

    def resize_with_padding(self, origin_image):
        h, w, _ = origin_image.shape
        self.scale = self.img_size / max(h, w)
        new_h, new_w = int(h * self.scale), int(w * self.scale)
        self.pad_top = (self.img_size - new_h) // 2
        self.pad_left = (self.img_size - new_w) // 2

        resized_image = cv2.resize(origin_image, (new_w, new_h))
        padded_image = cv2.copyMakeBorder(
            resized_image,
            top=self.pad_top,
            bottom=self.img_size - new_h - self.pad_top,
            left=self.pad_left,
            right=self.img_size - new_w - self.pad_left,
            borderType=cv2.BORDER_CONSTANT,
            value=(0, 0, 0),
        )

        return padded_image

    def expand_dims(self, padded_image):
        return np.expand_dims(padded_image, 0)

    # Post processing functions taken from https://github.com/airockchip/rknn_model_zoo
    def filter_boxes(self, boxes, box_confidences, box_class_probs):
        """Filter boxes with object threshold."""
        box_confidences = box_confidences.reshape(-1)
        candidate, class_num = box_class_probs.shape

        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)

        _class_pos = np.where(class_max_score * box_confidences >= self.threshold)
        scores = (class_max_score * box_confidences)[_class_pos]

        boxes = boxes[_class_pos]
        classes = classes[_class_pos]

        return boxes, classes, scores

    def nms_boxes(self, boxes, scores):
        """Suppress non-maximal boxes.
        # Returns
            keep: ndarray, index of effective boxes.
        """
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]

        areas = w * h
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1

            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= self.iou_threshold)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep

    def dfl(self, position):
        # Distribution Focal Loss (DFL)
        x = np.array(position, dtype=np.float32)
        n, c, h, w = x.shape
        p_num = 4
        mc = c // p_num
        y = x.reshape(n, p_num, mc, h, w)

        max_values = np.max(y, axis=2, keepdims=True)
        exp_values = np.exp(y - max_values)
        y = exp_values / np.sum(exp_values, axis=2, keepdims=True)

        acc_matrix = np.arange(mc, dtype=np.float32).reshape(1, 1, mc, 1, 1)
        y = np.sum(y * acc_matrix, axis=2)

        return y

    def box_process(self, position):
        grid_h, grid_w = position.shape[2:4]
        col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
        col = col.reshape(1, 1, grid_h, grid_w)
        row = row.reshape(1, 1, grid_h, grid_w)
        grid = np.concatenate((col, row), axis=1)
        stride = np.array([self.img_size / grid_w, self.img_size / grid_h]).reshape(
            1, 2, 1, 1
        )

        position = self.dfl(position)
        box_xy = grid + 0.5 - position[:, 0:2, :, :]
        box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
        xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)

        return xyxy

    def post_process(self, input_data):
        boxes, scores, classes_conf = [], [], []
        default_branch = 3
        pair_per_branch = len(input_data) // default_branch
        # Python 忽略 score_sum 输出
        for i in range(default_branch):
            boxes.append(self.box_process(input_data[pair_per_branch * i]))
            classes_conf.append(input_data[pair_per_branch * i + 1])
            scores.append(
                np.ones_like(
                    input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32
                )
            )

        def sp_flatten(_in):
            ch = _in.shape[1]
            _in = _in.transpose(0, 2, 3, 1)
            return _in.reshape(-1, ch)

        boxes = [sp_flatten(_v) for _v in boxes]
        classes_conf = [sp_flatten(_v) for _v in classes_conf]
        scores = [sp_flatten(_v) for _v in scores]

        boxes = np.concatenate(boxes)
        classes_conf = np.concatenate(classes_conf)
        scores = np.concatenate(scores)

        # filter according to threshold
        boxes, classes, scores = self.filter_boxes(boxes, scores, classes_conf)

        # nms
        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c = classes[inds]
            s = scores[inds]
            keep = self.nms_boxes(b, s)

            if len(keep) != 0:
                nboxes.append(b[keep])
                nclasses.append(c[keep])
                nscores.append(s[keep])

        if not nclasses and not nscores:
            return None, None, None

        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)

        boxes[:, 0] = (boxes[:, 0] - self.pad_left) / self.scale
        boxes[:, 1] = (boxes[:, 1] - self.pad_top) / self.scale
        boxes[:, 2] = (boxes[:, 2] - self.pad_left) / self.scale
        boxes[:, 3] = (boxes[:, 3] - self.pad_top) / self.scale

        return boxes, classes, scores
