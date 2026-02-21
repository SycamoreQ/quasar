ThisBuild / version := "0.1.0-SNAPSHOT"
ThisBuild / scalaVersion := "2.13.12"

ThisBuild / resolvers ++= Seq(
  "Sonatype OSS Releases" at "https://s01.oss.sonatype.org/content/repositories/releases"
)

lazy val root = (project in file("."))
  .settings(
    name := "accelerator",
    libraryDependencies ++= Seq(
      "org.chipsalliance" %% "chisel" % "6.4.0",
      "org.scalatest" %% "scalatest" % "3.2.19" % "test"
    )
  )