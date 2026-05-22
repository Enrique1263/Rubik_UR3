import json
import math
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox


class CircleEditor:
    def __init__(self, root, image_path=None):
        self.root = root
        self.root.title("Circle Editor")

        self.canvas = tk.Canvas(root, bg="black", cursor="cross")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.image_path = image_path
        self.photo = None
        self.bg_image_id = None

        self.circles = []
        self.pending_center = None
        self.preview_items = []

        self.point_radius = 3

        self._build_menu()
        self._bind_events()
        self._load_background_image(image_path)

    def _build_menu(self):
        menubar = tk.Menu(self.root)

        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Abrir imagen", command=self.open_image)
        file_menu.add_command(label="Guardar JSON", command=self.save_json)
        file_menu.add_command(label="Cargar JSON", command=self.load_json)
        file_menu.add_separator()
        file_menu.add_command(label="Salir", command=self.root.quit)

        menubar.add_cascade(label="Archivo", menu=file_menu)
        self.root.config(menu=menubar)

    def _bind_events(self):
        self.canvas.bind("<Button-1>", self.on_left_click)
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.root.bind("<KeyPress-z>", self.undo_last)
        self.root.bind("<KeyPress-Z>", self.undo_last)
        self.root.bind("<Escape>", self.cancel_pending)

    def _load_background_image(self, image_path):
        if not image_path:
            return

        try:
            self.photo = tk.PhotoImage(file=image_path)
            self.canvas.config(width=self.photo.width(), height=self.photo.height())
            self.canvas.delete("background")
            self.bg_image_id = self.canvas.create_image(
                0, 0, anchor=tk.NW, image=self.photo, tags="background"
            )
            self.canvas.tag_lower("background")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir la imagen:\n{e}")

    def open_image(self):
        path = filedialog.askopenfilename(
            title="Selecciona una imagen",
            filetypes=[
                ("Imágenes", "*.png *.gif *.ppm *.pgm"),
                ("PNG", "*.png"),
                ("GIF", "*.gif"),
                ("Todos los archivos", "."),
            ],
        )
        if path:
            self.image_path = path
            self._load_background_image(path)
            self.redraw_all()

    def on_left_click(self, event):
        x, y = event.x, event.y

        if self.pending_center is None:
            self.pending_center = (x, y)
            self.draw_pending_point()
        else:
            cx, cy = self.pending_center
            radius = math.hypot(x - cx, y - cy)

            self.circles.append({
                "id": len(self.circles) + 1,
                "center": {"x": cx, "y": cy},
                "radius": radius,
            })

            self.pending_center = None
            self.clear_preview()
            self.redraw_all()

    def on_mouse_move(self, event):
        if self.pending_center is None:
            return

        cx, cy = self.pending_center
        radius = math.hypot(event.x - cx, event.y - cy)

        self.clear_preview()

        point = self.canvas.create_oval(
            cx - self.point_radius,
            cy - self.point_radius,
            cx + self.point_radius,
            cy + self.point_radius,
            fill="yellow",
            outline="yellow",
            tags="preview",
        )
        circle = self.canvas.create_oval(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            outline="yellow",
            width=2,
            dash=(6, 4),
            tags="preview",
        )

        self.preview_items = [point, circle]

    def draw_pending_point(self):
        self.clear_preview()
        if self.pending_center is None:
            return

        cx, cy = self.pending_center
        point = self.canvas.create_oval(
            cx - self.point_radius,
            cy - self.point_radius,
            cx + self.point_radius,
            cy + self.point_radius,
            fill="yellow",
            outline="yellow",
            tags="preview",
        )
        self.preview_items = [point]

    def clear_preview(self):
        self.canvas.delete("preview")
        self.preview_items = []

    def redraw_all(self):
        self.canvas.delete("circle")
        self.canvas.delete("label")

        for idx, circle in enumerate(self.circles, start=1):
            cx = circle["center"]["x"]
            cy = circle["center"]["y"]
            r = circle["radius"]

            self.canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                outline="red",
                width=2,
                tags="circle",
            )
            self.canvas.create_oval(
                cx - self.point_radius,
                cy - self.point_radius,
                cx + self.point_radius,
                cy + self.point_radius,
                fill="red",
                outline="red",
                tags="circle",
            )
            self.canvas.create_text(
                cx + 8,
                cy + 8,
                text=str(idx),
                fill="white",
                anchor=tk.NW,
                tags="label",
            )

        if self.bg_image_id is not None:
            self.canvas.tag_lower("background")

    def undo_last(self, event=None):
        if self.pending_center is not None:
            self.pending_center = None
            self.clear_preview()
            return

        if self.circles:
            self.circles.pop()
            for idx, circle in enumerate(self.circles, start=1):
                circle["id"] = idx
            self.redraw_all()

    def cancel_pending(self, event=None):
        self.pending_center = None
        self.clear_preview()

    def save_json(self):
        path = filedialog.asksaveasfilename(
            title="Guardar círculos",
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return

        data = {
            "image_path": os.path.abspath(self.image_path) if self.image_path else None,
            "circles": self.circles,
        }

        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            messagebox.showinfo("Guardado", f"JSON guardado en:\n{path}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo guardar el JSON:\n{e}")

    def load_json(self):
        path = filedialog.askopenfilename(
            title="Cargar círculos",
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.circles = data.get("circles", [])
            for idx, circle in enumerate(self.circles, start=1):
                circle["id"] = idx

            image_path = data.get("image_path")
            if image_path and os.path.exists(image_path):
                self.image_path = image_path
                self._load_background_image(image_path)

            self.pending_center = None
            self.clear_preview()
            self.redraw_all()

            messagebox.showinfo("Cargado", f"JSON cargado desde:\n{path}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo cargar el JSON:\n{e}")


def main():
    image_path = sys.argv[1] if len(sys.argv) > 1 else None

    root = tk.Tk()
    app = CircleEditor(root, image_path=image_path)
    root.mainloop()


if __name__ == "__main__":
    main()