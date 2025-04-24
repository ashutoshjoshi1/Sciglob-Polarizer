from PyQt5.QtCore import QObject, pyqtSignal, QTimer, QDateTime
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit
import pyqtgraph as pg
import os
from drivers.spectrometer import connect_spectrometer, prepare_measurement, AVS_MeasureCallbackFunc, AVS_MeasureCallback, StopMeasureThread
import numpy as np

class SpectrometerController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.groupbox=QGroupBox("Spectrometer"); self.groupbox.setObjectName("specGroup")
        v=QVBoxLayout(); h=QHBoxLayout()
        self.toggle_btn=QPushButton("Start Saving"); self.toggle_btn.setEnabled(False);self.toggle_btn.clicked.connect(self.toggle)
        h.addWidget(self.toggle_btn)
        self.conn_btn=QPushButton("Connect");self.conn_btn.clicked.connect(self.connect)
        h.addWidget(self.conn_btn)
        self.start_btn=QPushButton("Start");self.start_btn.setEnabled(False);self.start_btn.clicked.connect(self.start)
        h.addWidget(self.start_btn)
        self.stop_btn=QPushButton("Stop");self.stop_btn.setEnabled(False);self.stop_btn.clicked.connect(self.stop)
        h.addWidget(self.stop_btn)
        self.save_btn=QPushButton("Save");self.save_btn.setEnabled(False);self.save_btn.clicked.connect(self.save)
        h.addWidget(self.save_btn)
        v.addLayout(h)
        pg.setConfigOption('background','w');pg.setConfigOption('foreground','k')
        self.plot=pg.PlotWidget();self.plot.setLabel('bottom','Wavelength','nm');self.plot.setLabel('left','Intensity','cts');self.plot.showGrid(x=True,y=True)
        self.curve=self.plot.plot([],[],pen=pg.mkPen('#2986cc'))
        v.addWidget(self.plot);
        self.groupbox.setLayout(v)
        self._ready=False; self.handle=None; self.wls=[]; self.intens=[]; self.npix=0; self.csv_dir="data";os.makedirs(self.csv_dir,exist_ok=True)

    def connect(self):
        self.status_signal.emit("Conn spectrometer...")
        try: h,wls,npix,ser=connect_spectrometer()
        except Exception as e: return self.status_signal.emit(str(e))
        self.handle=h; self.wls=wls.tolist() if isinstance(wls,np.ndarray) else wls; self.npix=npix
        self.start_btn.setEnabled(True); self._ready=True
        self.status_signal.emit(f"Ready (SN={ser})")

    def start(self):
        if not self._ready: return self.status_signal.emit("Not ready")
        res=prepare_measurement(self.handle,self.npix,50.0,1)
        if res!=0: return self.status_signal.emit(f"Prep err {res}")
        self.measure_active=True
        self.cb=AVS_MeasureCallbackFunc(self._cb)
        err=AVS_MeasureCallback(self.handle,self.cb,-1)
        if err!=0: return self.status_signal.emit(f"Callback err {err}")
        self.start_btn.setEnabled(False); self.stop_btn.setEnabled(True)
        self.timer=QTimer(self);self.timer.timeout.connect(self._upd);self.timer.start(200)
        self.status_signal.emit("Measuring...")

    def _cb(self,pd,pu):
        if pu[0]==0:
            _,data=connect_spectrometer.AVS_GetScopeData(self.handle)
            full=[0.0]*self.npix;full[:len(data)]=data; self.intens=full; self.save_btn.setEnabled(True);self.toggle_btn.setEnabled(True)
        else: self.status_signal.emit(f"Spectro err {pu[0]}")

    def _upd(self):
        if not hasattr(self,'intens'): return
        self.curve.setData(self.wls,self.intens)

    def stop(self):
        if not getattr(self,'measure_active',False): return
        self.stop_btn.setEnabled(False)
        th=StopMeasureThread(self.handle,parent=self); th.finished_signal.connect(self._stopped); th.start()

    def _stopped(self):
        self.measure_active=False;self.start_btn.setEnabled(True);self.status_signal.emit("Stopped")

    def save(self):
        ts=QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
        fn=os.path.join(self.csv_dir,f"snapshot_{ts}.csv")
        try:
            with open(fn,'w') as f:
                f.write("WL,Int\n")
                for wl,inten in zip(self.wls,self.intens):
                    if inten: f.write(f"{wl},{inten}\n")
            self.status_signal.emit(f"Saved {fn}")
        except Exception as e: self.status_signal.emit(f"Save err {e}")

    def toggle(self): pass  # implement continuous
    def is_ready(self): return self._ready