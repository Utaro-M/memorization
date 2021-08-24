(defun quick-sort-vector-list(seq &key(axis 0))
  ;; (format t "axis =~A~%" axis)
  (if (< (length seq) 2)
      (return-from quick-sort-vector-list seq :axis axis))
  (let ((pivot (elt (elt seq 0) axis))
        (pivot-vec (elt seq 0) )
        (left nil)
        (right nil))

  (dotimes (i (- (length seq) 1))
    (if (< (elt (elt seq (+ i 1)) axis) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  ;; (format t "left = ~A~%" left)
  ;; (format t "right = ~A~%" right)
  (setq left (quick-sort-vector-list left :axis axis))
  (setq right (quick-sort-vector-list right :axis axis))
  (setq fwd (append left (list pivot-vec)))
  (setq ret (append fwd right ))
  ret
  ))

(defun quick-sort(seq)
    (if (< (length seq) 1)
      (return-from quick-sort seq))
  (let ((pivot (elt seq 0))
        ( left nil)
        ( right nil))

  (dotimes (i (- (length seq) 1))
    ;; (print i)
    (if (< (elt seq (+ i 1)) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  ;; (format t "left = ~A~%" left)
  ;; (format t "right = ~A~%" right)
  (setq left (quick-sort left))
  (setq right (quick-sort right))
  (setq fwd (append left (list pivot) ))
  (setq ret (append fwd right ))
  ret
  ))