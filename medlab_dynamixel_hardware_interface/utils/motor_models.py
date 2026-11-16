from dataclasses import dataclass
from pathlib import Path

OPERATING_MODES = {
    "current": 0,
    "velocity": 1,
    "position": 3,
    "extended_position": 4,
    "current_based_position": 5,
    "pwm": 16,
}


@dataclass(frozen=True)
class Register:
    address: int
    size: int


@dataclass(frozen=True)
class UnitInfo:
    scale: float
    unit: str
    signed: bool
    offset: float


class MotorModel:
    def __init__(self, model_number, model_file):
        self.model_number = model_number
        self.model_file = Path(model_file)
        self.registers = {}
        self.units = {}
        self._load()

    def _load(self):
        section = None

        with self.model_file.open() as file:
            for line in file:
                line = line.strip()

                if not line or line.startswith("#"):
                    continue

                if line.startswith("[") and line.endswith("]"):
                    section = line[1:-1].strip().lower()
                    continue

                fields = line.split("\t")

                if section == "control table":
                    if fields[0] == "Address":
                        continue

                    address = int(fields[0])
                    size = int(fields[1])
                    name = fields[2].strip()

                    self.registers[name] = Register(address, size)

                elif section == "unit info":
                    if fields[0] == "Data Name":
                        continue

                    name = fields[0].strip()

                    self.units[name] = UnitInfo(
                        scale=float(fields[1]),
                        unit=fields[2].strip(),
                        signed=fields[3].strip().lower() == "signed",
                        offset=float(fields[4]),
                    )

    def get_register(self, name):
        if name not in self.registers:
            raise KeyError(
                f"Register '{name}' not found in "
                f"{self.model_file.name}"
            )

        return self.registers[name]

    def get_unit(self, name):
        if name not in self.units:
            raise KeyError(
                f"Unit information for '{name}' not found"
            )

        return self.units[name]

    def convert(self, name, raw_value):
        register = self.get_register(name)
        unit = self.get_unit(name)

        if unit.signed:
            bits = register.size * 8
            mask = (1 << bits) - 1
            raw_value &= mask
            sign_bit = 1 << (bits - 1)
            if raw_value & sign_bit:
                raw_value -= 1 << bits

        return raw_value * unit.scale + unit.offset


class MotorModelDatabase:
    def __init__(self, models_dir):
        self.models_dir = Path(models_dir)
        self.model_files = {}
        self.cache = {}
        self._load_index()

    def _load_index(self):
        index = self.models_dir / "dynamixel.model"

        with index.open() as file:
            for line in file:
                line = line.strip()

                if not line or line.startswith("#"):
                    continue

                fields = line.split()

                if fields[0] == "Number":
                    continue

                if len(fields) < 2:
                    continue

                model_number = int(fields[0])
                filename = fields[1]

                self.model_files[model_number] = filename

    def get_model(self, model_number):
        if model_number not in self.model_files:
            raise ValueError(
                f"Unsupported Dynamixel model: {model_number}"
            )

        if model_number not in self.cache:
            filename = self.model_files[model_number]
            path = self.models_dir / filename

            if not path.is_file():
                raise FileNotFoundError(
                    f"Missing model definition: {path}"
                )

            self.cache[model_number] = MotorModel(
                model_number, path
            )

        return self.cache[model_number]
