import sys
import os
import tempfile
import json
import shutil
from pathlib import Path
from datetime import datetime
import re
import gpxpy
import piexif

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget, 
    QPushButton, QFileDialog, QMessageBox, QSplitter, QLabel, 
    QProgressBar, QScrollArea, QGridLayout, QCheckBox
)
from PySide6.QtCore import Qt, QUrl, QSize, QByteArray, QBuffer, QIODevice
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtGui import QPixmap, QIcon

# Constants
MAX_NATIVE_ZOOM = 19 # Limit is 19 before errors occur
MAX_ZOOM = 20 # >19 possible
PHOTO_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.tiff', '.tif', '.raw'}
THUMBNAIL_SIZE = (100, 100)
MAP_THUMBNAIL_SIZE = (50, 50)

os.environ["QTWEBENGINE_REMOTE_DEBUGGING"] = "12345"

def parse_exif_datetime(dt_str):
    """Parse EXIF DateTimeOriginal with optional fractional seconds"""
    match = re.match(r'(\d{4}):(\d{2}):(\d{2}) (\d{2}):(\d{2}):(\d{2})(\.\d+)?', dt_str)
    if match == None:
        return None
    
    year, month, day, hour, minute, second = map(int, match.groups()[:6])
    microsecond = int(float(match.group(7) or 0) * 1000000)
    return datetime(year, month, day, hour, minute, second, microsecond)

def decimal_to_dms(decimal_degree):
    """Convert decimal degrees to DMS format for GPS EXIF"""
    decimal_degree = abs(decimal_degree)
    degrees = int(decimal_degree)
    minutes_float = (decimal_degree - degrees) * 60
    minutes = int(minutes_float)
    seconds = (minutes_float - minutes) * 60
    
    return (
        (degrees, 1),
        (minutes, 1), 
        (int(seconds * 1000000), 1000000)
    )

def get_photo_timestamp(image_path: str):
    """Extract timestamp from image EXIF data"""
    try:
        exif_dict = piexif.load(image_path)
        
        # First try DateTimeOriginal in the Exif IFD
        if 36867 in exif_dict['Exif']:
            datetime_bytes = exif_dict['Exif'][36867]
            datetime_str = datetime_bytes.decode('utf-8')
            return parse_exif_datetime(datetime_str)
        
        # Fallback to DateTime in 0th IFD
        elif 306 in exif_dict['0th']:
            datetime_bytes = exif_dict['0th'][306]
            datetime_str = datetime_bytes.decode('utf-8')
            return parse_exif_datetime(datetime_str)
            
    except Exception as e:
        print(f"Error extracting timestamp from '{image_path}': {e}")
    
    return None

def add_gps_to_exif(image_path, latitude, longitude, create_backup=True):
    """Add GPS coordinates to image EXIF data"""
    try:
        if create_backup:
            backup_path = image_path.with_suffix(f'{image_path.suffix}.backup')
            shutil.copy2(image_path, backup_path)
        
        # Handle JPEG files with piexif
        if image_path.suffix.lower() in ['.jpg', '.jpeg']:
            try:
                exif_dict = piexif.load(str(image_path))
            except:
                exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "thumbnail": None}
            
            if "GPS" not in exif_dict or exif_dict["GPS"] is None:
                exif_dict["GPS"] = {}
            
            lat_dms = decimal_to_dms(abs(latitude))
            lon_dms = decimal_to_dms(abs(longitude))
            
            exif_dict["GPS"].update({
                piexif.GPSIFD.GPSLatitude: lat_dms,
                piexif.GPSIFD.GPSLatitudeRef: b'N' if latitude >= 0 else b'S',
                piexif.GPSIFD.GPSLongitude: lon_dms,
                piexif.GPSIFD.GPSLongitudeRef: b'E' if longitude >= 0 else b'W',
                piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0)
            })
            
            piexif.insert(piexif.dump(exif_dict), str(image_path))
            return True
        
    except Exception as e:
        print(f"Error adding GPS to {image_path}: {e}")
        return False

class MapGenerator:
    """Handles map HTML generation and JavaScript operations"""
    
    @staticmethod
    def generate_html(points=None, center_lat=40.7128, center_lon=-74.0060):
        """Generate complete HTML for the map"""
        css_links = '<link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />'
        js_links = '<script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>'
        
        if points:
            js_links += '\n<script src="https://unpkg.com/leaflet-polylinedecorator@1.6.0/dist/leaflet.polylineDecorator.js"></script>'
        
        gpx_code = MapGenerator._get_gpx_js(points) if points else "window.leafletMap = map;"
        
        return f"""<!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            {css_links}
            <style>
                body {{ margin: 0; padding: 0; }}
                #map {{ height: 100vh; width: 100vw; }}
                .error {{ padding: 20px; background: #ffebee; color: #c62828; }}
                .photo-marker {{
                    border: 3px solid white; border-radius: 10px; box-shadow: 0 2px 4px rgba(0,0,0,0.2);
                    position: relative; background: red; width: 50px; height: 50px;
                }}
                .photo-marker::after {{
                    content: ''; position: absolute; bottom: -9px; left: 50%; transform: translateX(-50%);
                    width: 0; height: 0; border-left: 7px solid transparent; border-right: 7px solid transparent;
                    border-top: 9px solid white;
                }}
            </style>
        </head>
        <body>
            <div id="map">Loading map...</div>
            {js_links}
            <script>
                function initMap() {{
                    try {{
                        if (typeof L === 'undefined') throw new Error('Leaflet library not loaded');
                        
                        var map = L.map('map').setView([{center_lat}, {center_lon}], {2 if points is None else 13});
                        L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                            attribution: '© OpenStreetMap contributors',
                            maxNativeZoom: {MAX_NATIVE_ZOOM}, maxZoom: {MAX_ZOOM}
                        }}).addTo(map);
                        
                        {gpx_code}
                        
                        window.addPhotoMarker = function(lat, lon, thumbnail_base64, filename, timestamp) {{
                            var html = `<div class="photo-marker"><img src="${{thumbnail_base64}}" style="width: 50px; height: 50px; border-radius: 8px;" /></div>`;
                            return L.marker([lat, lon], {{
                                icon: L.divIcon({{ className: '', html: html, iconSize: [56, 62], iconAnchor: [28, 62], popupAnchor: [0, -62] }})
                            }}).addTo(map).bindPopup('<div style="text-align:center;">' + filename + '<br>' + timestamp + '</div>');
                        }};
                        
                    }} catch(e) {{
                        document.getElementById('map').innerHTML = '<div class="error">Map failed to load: ' + e.message + '</div>';
                        console.error('Map error:', e);
                    }}
                }}
                
                function attemptMapInit() {{
                    if (typeof L !== 'undefined' && !window.leafletMap) initMap();
                    else if (typeof L === 'undefined') setTimeout(attemptMapInit, 100);
                }}
                
                if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', attemptMapInit);
                else attemptMapInit();
                window.addEventListener('load', () => setTimeout(attemptMapInit, 500));
            </script>
        </body>
        </html>"""
    
    @staticmethod
    def _get_gpx_js(points):
        """Generate JavaScript code for GPX track display"""
        points_json = json.dumps([[p.latitude, p.longitude] for p in points])
        types_json = json.dumps([[p.type] for p in points])
        times_json = json.dumps([[str(p.time.strftime("%H:%M:%S"))] for p in points])
        return f"""
                var points = {points_json};
                var types = {types_json};
                var times = {times_json};
                if (points.length > 0) {{
                    var polyline = L.polyline(points, {{ color: 'red', weight: 3, opacity: 0.8 }}).addTo(map);
                    var decorator = L.polylineDecorator(polyline, {{
                        patterns: [{{ offset: 30, repeat: 30, symbol: L.Symbol.arrowHead({{
                            pixelSize: 10, pathOptions: {{ fillOpacity: 1, weight: 0, color: 'red', fillColor: 'red' }}
                        }}) }}]
                    }}).addTo(map);
                    window.gpxPolyline = polyline;
                    L.marker(points[0]).addTo(map).bindPopup('<div style="text-align:center;">Start<br></div>' + times[0]);
                    if (points.length > 1) L.marker(points[points.length-1]).addTo(map).bindPopup('<div style="text-align:center;">End<br></div>' + times[points.length-1]);

                    for (let i = 0; i < types.length; i++){{
                        if (types[i][0] == 'waypoint'){{
                            L.marker(points[i]).addTo(map).bindPopup('<div style="text-align:center;">Waypoint<br></div>' + times[i]);
                        }}
                        if (types[i][0] == 'routepoint') {{
                            L.marker(points[i]).addTo(map).bindPopup('<div style="text-align:center;">Route Point<br></div>' + times[i]);
                        }}
                    }}

                    map.fitBounds(polyline.getBounds());
                }}
                window.leafletMap = map;
                """

class CustomPoint():
    def __init__(self, time, latitude, longitude, type):
        self.time = time
        self.latitude = latitude
        self.longitude = longitude
        self.type = type

class GPSPhotoTagger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.photos = []
        self.map_file_path = None
        self.initUI()
        self.gpx_data = None
        self.points_collection = []
        
    def initUI(self):
        """Initialize the user interface"""
        self.setWindowTitle('NiceGPX')
        self.setGeometry(100, 100, 1200, 800)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QHBoxLayout(central_widget)
        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter)
        
        splitter.addWidget(self._create_left_panel())
        splitter.addWidget(self._create_right_panel())
        splitter.setSizes([360, 840])
        
        self._create_empty_map()
        
    def _create_left_panel(self):
        """Create the left panel with photo controls"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Progress bar styling
        progress_style = """
            QProgressBar { border: 1px solid #c0c0c0; border-radius: 3px; background-color: #f0f0f0; text-align: center; }
            QProgressBar::chunk { background-color: #4CAF50; border-radius: 2px; }
        """

        # Photos section
        photos_label = QLabel("Photos")
        photos_label.setStyleSheet("font-weight: bold; font-size: 14px; margin: 10px 0;")
        layout.addWidget(photos_label)

        open_photos_btn = QPushButton("Open Photos Folder")
        open_photos_btn.clicked.connect(self._open_photos_folder)
        layout.addWidget(open_photos_btn)

        self.photos_progress = QProgressBar()
        self.photos_progress.setVisible(False)
        self.photos_progress.setStyleSheet(progress_style)
        layout.addWidget(self.photos_progress)

        # Photo scroll area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.photo_container = QWidget()
        self.grid_layout = QGridLayout(self.photo_container)
        self.empty_label = QLabel("Drag and drop photos in or open photos folder")
        self.empty_label.setStyleSheet("color: rgba(0, 0, 0, 128);")
        self.empty_label.setAlignment(Qt.AlignCenter)
        self.grid_layout.setSpacing(10)
        self.scroll_area.setWidget(self.photo_container)
        layout.addWidget(self.scroll_area)
        self.grid_layout.addWidget(self.empty_label, 0, 0, Qt.AlignCenter)

        # Backup copies checkbox
        self.backup_checkbox = QCheckBox("Create backup copies")
        self.backup_checkbox.setChecked(True)
        self.backup_checkbox.setToolTip("Create backup copies of original photos before modifying")
        layout.addWidget(self.backup_checkbox)

        # Process button
        self.process_btn = QPushButton("Add GPS to Photos")
        self.process_btn.clicked.connect(self._process_photos)
        self.process_btn.setEnabled(False)
        layout.addWidget(self.process_btn)

        self.process_progress = QProgressBar()
        self.process_progress.setVisible(False)
        self.process_progress.setStyleSheet(progress_style)
        layout.addWidget(self.process_progress)

        return widget
        
    def _create_right_panel(self):
        """Create the right panel with map controls"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Map buttons
        buttons_layout = QHBoxLayout()
        
        open_gpx_btn = QPushButton("Open GPX File")
        open_gpx_btn.clicked.connect(self._open_gpx_file)
        buttons_layout.addWidget(open_gpx_btn)

        self.fit_to_gpx_btn = QPushButton("Fit View to GPX")
        self.fit_to_gpx_btn.clicked.connect(self._fit_map_to_gpx)
        self.fit_to_gpx_btn.setEnabled(False)
        buttons_layout.addWidget(self.fit_to_gpx_btn)
        
        buttons_layout.addStretch()
        layout.addLayout(buttons_layout)

        # Map view
        self.map_view = QWebEngineView()
        from PySide6.QtWebEngineCore import QWebEngineSettings
        settings = self.map_view.page().settings()
        settings.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        layout.addWidget(self.map_view)
        
        return widget
    
    def _create_thumbnail(self, image_path, size=THUMBNAIL_SIZE):
        """Create thumbnail QIcon from image file"""
        try:
            pixmap = QPixmap(str(image_path))
            if pixmap.isNull():
                return None
            
            scaled_pixmap = pixmap.scaled(
                QSize(*size), Qt.AspectRatioMode.KeepAspectRatio, 
                Qt.TransformationMode.SmoothTransformation
            )
            return QIcon(scaled_pixmap)
        except Exception as e:
            print(f"Error creating thumbnail for {image_path}: {e}")
            return None

    def _fit_map_to_gpx(self):
        """Fit map view to GPX track bounds"""
        if self.gpx_data:
            js_code = """
            if (window.leafletMap && window.gpxPolyline) {
                window.leafletMap.fitBounds(window.gpxPolyline.getBounds());
            }
            """
            self.map_view.page().runJavaScript(js_code)
    
    def _create_empty_map(self):
        """Create and display empty map"""
        html_content = MapGenerator.generate_html()
        self._save_and_load_map(html_content)
        
    def _save_and_load_map(self, html_content):
        """Save HTML to temp file and load in web view"""
        self.map_file_path = os.path.join(tempfile.gettempdir(), 'gps_map.html')
        with open(self.map_file_path, 'w', encoding='utf-8') as f:
            f.write(html_content)
        self.map_view.load(QUrl.fromLocalFile(self.map_file_path))
        
    def _clear_gpx_data(self):
        self.gpx_data = None
        self.points_collection = []
    
    def _open_gpx_file(self):
        """Open and parse GPX file"""
        self._clear_gpx_data()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open GPX File", "", "GPX files (*.gpx);;All files (*.*)"
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as gpx_file:
                    self.gpx_data = gpxpy.parse(gpx_file)
                    self._display_gpx_on_map()
                    self._check_ready_to_process()
                    self.fit_to_gpx_btn.setEnabled(True)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load GPX file: {str(e)}")
                
    def _display_gpx_on_map(self):
        """Display GPX track on map"""
        if self.gpx_data == None:
            return
        
        try:
            for track in self.gpx_data.tracks:
                for segment in track.segments:
                    for point in segment.points:
                        self.points_collection.append(CustomPoint(point.time, point.latitude, point.longitude, 'trackpoint'))
        except: pass
        
        try:
            for waypoint in self.gpx_data.waypoints:
                self.points_collection.append(CustomPoint(waypoint.time, waypoint.latitude, waypoint.longitude, 'waypoint'))
        except: pass
        
        try:
            for route in self.gpx_data.routes:
                for routepoint in route.points:
                    self.points_collection.append(CustomPoint(routepoint.time, routepoint.latitude, routepoint.longitude, 'routepoint'))
        except: pass
        
        if len(self.points_collection) == 0:
            QMessageBox.warning(self, "Warning", "No track points found in GPX file")
            return
        
        lats = [p.latitude for p in self.points_collection]
        lons = [p.longitude for p in self.points_collection]
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)
        
        html_content = MapGenerator.generate_html(self.points_collection, center_lat, center_lon)
        self._save_and_load_map(html_content)
        
    def _open_photos_folder(self):
        """Open folder dialog and load photos"""
        folder_path = QFileDialog.getExistingDirectory(self, "Select Photos Folder")
        if folder_path:
            self._load_photos_from_folder(folder_path)
            
    def _load_photos_from_folder(self, folder_path):
        """Load and display photo thumbnails from folder"""
        self.photos = []
        folder = Path(folder_path)
        photo_files = [f for f in folder.iterdir() 
                      if f.suffix.lower() in PHOTO_EXTENSIONS and f.is_file()]
        
        if not photo_files:
            QMessageBox.warning(self, "Warning", "No photo files found in selected folder")
            return
        
        # Clear 'drag and drop' message
        self.grid_layout.removeWidget(self.empty_label)
        self.empty_label.deleteLater()

        # Set photo grid layout
        self.grid_layout.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        
        # Clear existing thumbnails
        while self.grid_layout.count():
            child = self.grid_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        # Show progress
        self.photos_progress.setVisible(True)
        self.photos_progress.setMaximum(len(photo_files))
        self.photos_progress.setValue(0)

        # Create thumbnails
        columns = 2
        for idx, file_path in enumerate(photo_files):
            self.photos.append(file_path)
            icon = self._create_thumbnail(file_path)
            
            if icon:
                container = QWidget()
                layout = QVBoxLayout(container)
                layout.setAlignment(Qt.AlignCenter)
                layout.setContentsMargins(0, 0, 0, 0)
                
                image_label = QLabel()
                image_label.setPixmap(icon.pixmap(100, 100))
                image_label.setAlignment(Qt.AlignCenter)
                image_label.setFixedSize(110, 110)
                
                filename_label = QLabel(file_path.name)
                filename_label.setAlignment(Qt.AlignCenter)
                filename_label.setWordWrap(True)
                filename_label.setStyleSheet("font-size: 10px; color: #555;")
                
                layout.addWidget(image_label)
                layout.addWidget(filename_label)
                self.grid_layout.addWidget(container, idx // columns, idx % columns)
            
            self.photos_progress.setValue(idx + 1)
            if idx % 10 == 0:  # Update UI less frequently
                QApplication.processEvents()
        
        self.photos_progress.setFormat(f"✓ {len(photo_files)}/{len(photo_files)} photos loaded")
        self._check_ready_to_process()

    def _check_ready_to_process(self):
        """Enable process button if both GPX and photos are loaded"""
        self.process_btn.setEnabled(self.gpx_data != None and len(self.photos) > 0)
        
    def _process_photos(self):
        """Process photos by adding GPS coordinates from GPX track"""
        if not self.gpx_data or not self.photos:
            QMessageBox.warning(self, "Warning", "Please load both GPX file and photos first")
            return
    
        # Confirm processing
        create_backup = self.backup_checkbox.isChecked()
        msg = f"This will modify the original photo files. "
        msg += "Backup copies will be created with .backup extension." if create_backup else "No backup copies will be created."
        msg += "\n\nDo you want to continue?"
        
        if QMessageBox.question(self, "Confirm Processing", msg, QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
    
        # Clear existing photo markers
        self.map_view.page().runJavaScript(
            "if (window.photoMarkers) { window.photoMarkers.forEach(m => m.remove()); } window.photoMarkers = [];"
        )
    
        if not self.points_collection:
            QMessageBox.warning(self, "Warning", "No track points with timestamps found in GPX file.")
            return
    
        # Setup progress tracking
        self.process_progress.setMaximum(len(self.photos))
        self.process_progress.setValue(0)
        self.process_progress.setFormat("Processing %v of %m photos...")
        self.process_progress.setVisible(True)
        
        processed_count = 0
        
        for index, photo_path in enumerate(self.photos):
            photo_timestamp = get_photo_timestamp(str(photo_path))
            
            if photo_timestamp == None:
                print(f"Skipping {photo_path.name} - no timestamp found")
                self.process_progress.setValue(index + 1)
                continue

            # Find closest GPX point
            closest_point = min(self.points_collection, key=lambda p: abs(photo_timestamp.timestamp() - p.time.timestamp()))
            lat, lon = closest_point.latitude, closest_point.longitude

            # Add GPS to EXIF
            if add_gps_to_exif(photo_path, lat, lon, create_backup):
                processed_count += 1
                print(f"Added GPS ({lat:.6f}, {lon:.6f}) to {photo_path.name}")
                
                # Add map marker
                try:
                    pixmap = QPixmap(str(photo_path))
                    if not pixmap.isNull():
                        pixmap = pixmap.scaled(QSize(*MAP_THUMBNAIL_SIZE), 
                                             Qt.AspectRatioMode.KeepAspectRatio, 
                                             Qt.TransformationMode.FastTransformation)
                        
                        buffer = QByteArray()
                        buf = QBuffer(buffer)
                        buf.open(QIODevice.WriteOnly)
                        pixmap.save(buf, "JPEG", 70)
                        base64_data = 'data:image/jpeg;base64,' + str(buffer.toBase64(), 'utf-8')

                        js_code = f"""
                        if (!window.photoMarkers) window.photoMarkers = [];
                        var marker = window.addPhotoMarker({lat}, {lon}, '{base64_data}', '{photo_path.name}', '{photo_timestamp}');
                        window.photoMarkers.push(marker);
                        """
                        self.map_view.page().runJavaScript(js_code)
                except Exception as e:
                    print(f"Error creating map marker for {photo_path.name}: {e}")
            else:
                print(f"Failed to add GPS data to {photo_path.name}")

            self.process_progress.setValue(index + 1)
            if index % 10 == 0:  # Update UI less frequently
                QApplication.processEvents()
    
        # Show completion
        self.process_progress.setFormat(f"✓ {processed_count}/{len(self.photos)} photos processed")
        backup_msg = "Backup files created with .backup extension." if create_backup else ""
        QMessageBox.information(self, "Processing Complete", 
                              f"Successfully added GPS data to {processed_count} out of {len(self.photos)} photos.\n{backup_msg}")

def main():
    app = QApplication(sys.argv)
    window = GPSPhotoTagger()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()