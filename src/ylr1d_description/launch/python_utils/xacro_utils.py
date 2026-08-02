"""xacro → URDF 公共导入逻辑。

ylr1d_description 是模型资产单一来源，各包 launch 通过这里把
xacro + config/*.yaml + controllers.yaml 统一处理成 URDF 字符串
与临时 urdf 文件，避免在多个 launch 里重复同一段管道。
"""

import os
import re
import tempfile

import xacro
import yaml

# YAML 配置名 → config/*.yaml（${name.key} 的预替换来源）
_BASE_CONFIGS = ["links", "colors", "limits", "calibration", "dynamics", "sensors"]


def resolve_yaml_refs(content: str, config_dir: str) -> str:
    """把 ${links.X.Y} / ${colors.X} / ${limits.X.Y} 等占位符预替换为 yaml 字面量。

    模型 xacro 引用 config/*.yaml 的数值，但 xacro 本身不加载 yaml，
    故先在此预替换成字面量再交 xacro 展开。额外加载 config/sensors/*.yaml
    （超集，desc 版不引用 sensors 值，加载了也无害）。
    纯变量名（如 ${prefix}）保留给 xacro 自己解析。
    """
    configs = {}
    for name in _BASE_CONFIGS:
        path = os.path.join(config_dir, f"{name}.yaml")
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            if data is not None:
                configs[name] = data

    sensors_dir = os.path.join(config_dir, "sensors")
    if os.path.isdir(sensors_dir):
        for fname in sorted(os.listdir(sensors_dir)):
            if fname.endswith(".yaml"):
                name = fname[:-5]  # strip .yaml → e.g. "rgb_camera"
                path = os.path.join(sensors_dir, fname)
                with open(path) as f:
                    data = yaml.safe_load(f)
                if data is not None:
                    configs[name] = data

    def _resolve(match):
        expr = match.group(1).strip()
        # Leave simple variable names (prefix, ...) for xacro to handle
        if re.match(r'^[a-zA-Z_]\w*$', expr):
            return match.group(0)
        parts = expr.split(".")
        if parts[0] in configs:
            try:
                val = configs[parts[0]]
                for p in parts[1:]:
                    val = val[p]
                return str(val)
            except (KeyError, TypeError):
                pass
        return match.group(0)  # keep as-is if unresolvable

    return re.sub(r'\$\{([^}]+)\}', _resolve, content)


def process_xacro_to_urdf(xacro_path: str, config_dir: str,
                          controllers_yaml: str | None = None,
                          transforms: tuple = ()) -> tuple[str, str]:
    """把 xacro 处理成 URDF 字符串与临时 urdf 文件。

    :param xacro_path: xacro 文件绝对路径
    :param config_dir: config/ 目录（供 ${links.X.Y} 预替换）
    :param controllers_yaml: ${controllers_yaml_path} 的注入值；None 则不注入
    :param transforms: 预处理链 [(str)->str]，在 xacro 展开前依次作用于内容
    :return: (robot_desc, urdf_tmp_path) —— urdf 临时文件故意不删，
             由调用方在 spawn 完成后清理（spawn 是异步延迟执行）
    """
    with open(xacro_path) as f:
        raw = f.read()
    resolved = resolve_yaml_refs(raw, config_dir)
    if controllers_yaml:
        resolved = resolved.replace("${controllers_yaml_path}", controllers_yaml)
    for t in transforms:
        resolved = t(resolved)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False)
    tmp.write(resolved)
    tmp.close()
    try:
        doc = xacro.process_file(tmp.name)
        robot_desc = doc.toxml()
    finally:
        os.unlink(tmp.name)

    urdf_tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    urdf_tmp.write(robot_desc)
    urdf_tmp.close()
    return robot_desc, urdf_tmp.name
