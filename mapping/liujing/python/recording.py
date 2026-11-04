"""Bounded keyframe recorder. Producer shutdown precedes explicit drain sentinel."""
import queue
import threading
import numpy as np


class KeyframeWriter:
    def __init__(self, folder, on_failure, capacity=4):
        self.folder=folder;self.on_failure=on_failure;self.queue=queue.Queue(maxsize=capacity)
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start();self.closed=False

    def submit(self,index,data):
        if self.closed:raise RuntimeError('recorder already closed')
        try:self.queue.put_nowait((index,data));return True
        except queue.Full:return False

    def _run(self):
        while True:
            item=self.queue.get()
            try:
                if item is None:return
                index,data=item;np.savez_compressed(self.folder/f'{index:06}.npz',**data)
            except Exception as exc:self.on_failure(exc)
            finally:self.queue.task_done()

    def close(self):
        if not self.closed:
            self.closed=True;self.queue.put(None);self.thread.join()
